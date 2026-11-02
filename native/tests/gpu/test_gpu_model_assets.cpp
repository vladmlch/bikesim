// native/tests/gpu/test_gpu_model_assets.cpp — C++23 unit test for native/src/gpu/model_assets.hpp.
// Real-model checks use tools/fp32_probe/artifacts/common (model.xml, hfield_data.bin,
// captured_bootstrap.json). Error paths use temp directories under /tmp/bike_orch/D/.
//
// Usage: bike_test_gpu_model_assets [case-name|--list]
//        (the model is located relative to BIKE_REPO_ROOT or the cwd)

#include "../../src/gpu/model_assets.hpp"
#include "../../src/platform/argv.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace assets = bike::gpu::assets;
namespace fs = std::filesystem;

namespace {

using TestFunction = void (*)(const fs::path &);

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

// Status comparison with the symbolic names in the message, replacing the old
// REQUIRE_STATUS macro's fprintf of got/want.
void require_status(assets::AssetStatus got, assets::AssetStatus want, std::string_view what) {
    require(got == want, std::string(what) + ": got " + assets::asset_status_name(got) +
                             ", want " + assets::asset_status_name(want));
}

// Model counts arrive as int/mjtSize; container subscripts and spans take
// size_t, so the explicit conversion lives in one place (same helper as the
// other gpu tests).
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

// The single raw mjModel pointer -> std::span boundary. MuJoCo owns these
// arrays for the model's lifetime and sizes them by the adjacent counts
// (nhfield, nhfielddata, ngeom); every access range in this file is wrapped
// once through this helper. The (pointer, count) construction itself is
// -Wunsafe-buffer-usage-in-container — suppressed here, scoped exactly like
// native/src/platform/argv.hpp and native/src/gpu/model_assets.hpp's
// detail::model_span (the header is not included just for this helper).
template <typename T>
[[nodiscard]] std::span<T> model_span(T *ptr, std::size_t count) noexcept {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    return std::span<T>(ptr, count);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
}

bool same_bits(double a, double b) {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool same_bits(float a, float b) {
    return std::bit_cast<std::uint32_t>(a) == std::bit_cast<std::uint32_t>(b);
}

constexpr const char *kRelModelPath = "tools/fp32_probe/artifacts/common/model.xml";

fs::path find_probe_model_xml() {
    std::vector<fs::path> candidates;
#ifdef BIKE_REPO_ROOT
    candidates.emplace_back(fs::path(BIKE_REPO_ROOT) / kRelModelPath);
#endif
    candidates.emplace_back(kRelModelPath);
    candidates.emplace_back(fs::path("..") / kRelModelPath);
    candidates.emplace_back(fs::path("../..") / kRelModelPath);
    candidates.emplace_back(fs::path("../../..") / kRelModelPath);
    for (const auto &p : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(p, ec)) {
            return fs::canonical(p, ec);
        }
    }
    return {};
}

std::string read_file(const fs::path &p) {
    std::ifstream f(p, std::ios::binary);
    require(f.is_open(), "cannot open " + p.string());
    const std::string bytes((std::istreambuf_iterator<char>(f)),
                            std::istreambuf_iterator<char>());
    return bytes;
}

void write_file(const fs::path &p, const std::string &bytes) {
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    require(f.is_open(), "cannot open " + p.string() + " for writing");
    f.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    require(f.good(), "short write to " + p.string());
}

fs::path scratch_root() {
    return {"/tmp/bike_orch/D/model_assets_test"};
}

fs::path fresh_dir(const std::string &name) {
    const fs::path dir = scratch_root() / name;
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    require(!ec, "cannot create scratch directory " + dir.string() + ": " + ec.message());
    return dir;
}

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;

ModelPtr load_model(const fs::path &xml) {
    std::array<char, 1024> error{};
    mjModel *raw = mj_loadXML(xml.string().c_str(), nullptr, error.data(),
                              static_cast<int>(error.size()));
    require(raw != nullptr, "mj_loadXML(" + xml.string() + "): " + error.data());
    return {raw, &mj_deleteModel};
}

// MuJoCo's hfield_data is float (see mjmodel.h), so comparisons are done in float.
bool all_equal_to(std::span<const float> values, float value) {
    return std::ranges::all_of(values, [value](float x) { return same_bits(x, value); });
}

void test_status_names(const fs::path & /*unused*/) {
    constexpr std::array<assets::AssetStatus, 7> all{{
        assets::AssetStatus::kOk,           assets::AssetStatus::kMissingFile,
        assets::AssetStatus::kSizeMismatch, assets::AssetStatus::kReadError,
        assets::AssetStatus::kParseError,   assets::AssetStatus::kDimMismatch,
        assets::AssetStatus::kNoHeightfield,
    }};
    for (const assets::AssetStatus s : all) {
        require(std::string_view{assets::asset_status_name(s)} != "unknown",
                "every AssetStatus maps to a real name");
    }
    require(std::string_view{assets::asset_status_name(assets::AssetStatus::kOk)} == "ok",
            "kOk names itself ok");
    require(assets::asset_dir("model.xml") == ".", "a bare file name maps to .");
    require(assets::asset_dir("") == ".", "an empty path maps to .");
    require(assets::asset_dir("/a/b/model.xml") == "/a/b", "asset_dir returns the parent");
}

// load_hfield_bin on the real model: zero-fill first so the loader must actually write every value.
void test_hfield_bin_real(const fs::path &xml) {
    const ModelPtr m = load_model(xml);
    require(m->nhfield == 1, "model carries exactly one heightfield");
    const mjtSize n = m->nhfielddata;
    require(n > 0, "model carries heightfield data");
    const auto count = static_cast<std::size_t>(n);

    const std::string bin = read_file(xml.parent_path() / assets::kHfieldBinName);
    require(bin.size() == count * sizeof(double),
            "hfield_data.bin holds exactly nhfielddata binary64 samples");
    std::vector<double> expect(count);
    // Byte copy into the doubles buffer runs span-to-span over std::byte views,
    // the same idiom load_hfield_bin uses — no pointer reinterpretation.
    const std::span<const std::byte> src = std::as_bytes(std::span{bin});
    const std::span<std::byte> dst = std::as_writable_bytes(std::span{expect});
    std::ranges::copy(src, dst.begin());

    const std::span<float> hfield = model_span(m->hfield_data, count);
    std::ranges::fill(hfield, -1.0f);
    require_status(assets::load_hfield_bin(m.get(), xml.string()), assets::AssetStatus::kOk,
                   "load_hfield_bin on the real model");
    for (std::size_t i = 0; i < count; ++i) {
        require(same_bits(hfield[i], static_cast<float>(expect[i])),
                "hfield_data[" + std::to_string(i) +
                    "] is the binary64 sample rounded to float");
    }
}

// Independent re-derivation of the cpu_batch_runner.cpp extraction, checked bitwise.
void test_extract_real(const fs::path &xml) {
    const ModelPtr m = load_model(xml);
    assets::Heightfield1D hf;
    require_status(assets::extract_heightfield_1d(m.get(), hf), assets::AssetStatus::kOk,
                   "extract_heightfield_1d on the real model");

    const std::size_t nhfield = static_cast<std::size_t>(m->nhfield);
    const std::size_t ngeom = static_cast<std::size_t>(m->ngeom);
    const std::span<const int> hfield_ncol = model_span(m->hfield_ncol, nhfield);
    const std::span<const mjtNum> hfield_size = model_span(m->hfield_size, 4 * nhfield);
    const std::span<const int> geom_type = model_span(m->geom_type, ngeom);
    const std::span<const mjtNum> geom_pos = model_span(m->geom_pos, 3 * ngeom);
    const std::span<const float> hfield_data =
        model_span(m->hfield_data, static_cast<std::size_t>(m->nhfielddata));

    const int ncol = hfield_ncol[0];
    require(hf.heights.size() == to_index(ncol), "heights carries ncol samples");

    const float half = static_cast<float>(hfield_size[0]);
    const float scale_z = static_cast<float>(hfield_size[2]);
    float ref_x_min = 0.0f;
    float ref_z = 0.0f;
    bool found = false;
    for (std::size_t g = 0; g < ngeom && !found; ++g) {
        if (geom_type[g] == mjGEOM_HFIELD) {
            ref_x_min = static_cast<float>(geom_pos[3 * g + 0]) - half;
            ref_z = static_cast<float>(geom_pos[3 * g + 2]);
            found = true;
        }
    }
    require(found, "model carries a heightfield geom");
    const float ref_total = (half > 0.0f) ? (2.0f * half) : (static_cast<float>(ncol - 1) * 0.005f);
    const float ref_dx = ref_total / static_cast<float>(std::max(ncol - 1, 1));
    for (int i = 0; i < ncol; ++i) {
        const float ref_h =
            static_cast<float>(hfield_data[to_index(i)]) * scale_z + ref_z;
        require(same_bits(hf.heights[to_index(i)], ref_h),
                "heights[" + std::to_string(i) + "] is hfield_data*scale_z + geom z");
    }
    require(same_bits(hf.x_min_m, ref_x_min), "x_min_m is geom x - hfield half-extent");
    require(same_bits(hf.dx_m, ref_dx), "dx_m is total x length / max(ncol - 1, 1)");

    // Known geometry from model.xml: size x-half 60 m, geom at x = 60 m, 24001 samples.
    require(ncol == 24001, "model.xml carries 24001 heightfield samples");
    require(same_bits(hf.x_min_m, 0.0f), "x_min_m is 0 for the known geometry");
    require(same_bits(hf.dx_m, 0.005f), "dx_m is 5 mm for the known geometry");

    m->nhfield = 0;
    require_status(assets::extract_heightfield_1d(m.get(), hf), assets::AssetStatus::kNoHeightfield,
                   "extract_heightfield_1d with nhfield = 0");
    m->nhfield = 1;
    require_status(assets::extract_heightfield_1d(nullptr, hf), assets::AssetStatus::kNoHeightfield,
                   "extract_heightfield_1d on a null model");
}

// Reference values from Python json (repr round-trips exactly): state[1], state[26], state[27], state[52].
void test_captured_real(const fs::path &xml) {
    std::vector<double> qpos{1.5};
    std::vector<double> qvel{2.5};
    require_status(assets::load_captured_initial_state(xml.string(), 26, 26, qpos, qvel),
                   assets::AssetStatus::kOk,
                   "load_captured_initial_state on the real bootstrap");
    require(qpos.size() == 26, "qpos carries nq samples");
    require(qvel.size() == 26, "qvel carries nv samples");
    require(same_bits(qpos[0], 2.0163204663006566), "qpos[0] is the captured state[1]");
    require(same_bits(qpos[25], 0.03599510121494982), "qpos[25] is the captured state[26]");
    require(same_bits(qvel[0], 0.058017614062661975), "qvel[0] is the captured state[27]");
    require(same_bits(qvel[25], 0.5931204424524291), "qvel[25] is the captured state[52]");
}

void test_hfield_errors(const fs::path &xml) {
    const ModelPtr m = load_model(xml);
    const mjtSize n = m->nhfielddata;
    const auto count = static_cast<std::size_t>(n);
    const std::string bin = read_file(xml.parent_path() / assets::kHfieldBinName);
    const std::span<float> hfield = model_span(m->hfield_data, count);

    // Missing file: untouched.
    const fs::path d_missing = fresh_dir("hfield_missing");
    std::ranges::fill(hfield, -7.0f);
    require_status(assets::load_hfield_bin(m.get(), (d_missing / "model.xml").string()),
                   assets::AssetStatus::kMissingFile, "load_hfield_bin with no file");
    require(all_equal_to(hfield, -7.0f), "a missing file leaves hfield_data untouched");

    // Truncated by one sample: untouched.
    const fs::path d_trunc = fresh_dir("hfield_truncated");
    write_file(d_trunc / assets::kHfieldBinName, bin.substr(0, bin.size() - sizeof(double)));
    require_status(assets::load_hfield_bin(m.get(), (d_trunc / "model.xml").string()),
                   assets::AssetStatus::kSizeMismatch, "load_hfield_bin on a truncated file");
    require(all_equal_to(hfield, -7.0f), "a truncated file leaves hfield_data untouched");

    // Oversized by one sample: untouched.
    const fs::path d_over = fresh_dir("hfield_oversized");
    write_file(d_over / assets::kHfieldBinName, bin + std::string(sizeof(double), '\0'));
    require_status(assets::load_hfield_bin(m.get(), (d_over / "model.xml").string()),
                   assets::AssetStatus::kSizeMismatch, "load_hfield_bin on an oversized file");
    require(all_equal_to(hfield, -7.0f), "an oversized file leaves hfield_data untouched");

    // A model without heightfield data needs no file and touches nothing.
    m->nhfielddata = 0;
    require_status(assets::load_hfield_bin(m.get(), (d_missing / "model.xml").string()),
                   assets::AssetStatus::kOk, "nhfielddata == 0 needs no file");
    m->nhfielddata = n;
    require(all_equal_to(hfield, -7.0f), "the no-data path leaves hfield_data untouched");

    require_status(assets::load_hfield_bin(nullptr, (d_missing / "model.xml").string()),
                   assets::AssetStatus::kNoHeightfield, "load_hfield_bin on a null model");
}

struct JsonCase {
    std::string_view name;
    std::string text;
    int nq;
    int nv;
    assets::AssetStatus want;
    std::vector<double> qpos_want;
    std::vector<double> qvel_want;
};

void run_json_case(const JsonCase &c, int index) {
    const std::string ctx = "case '" + std::string(c.name) + "': ";
    const fs::path dir = fresh_dir("json_case_" + std::to_string(index));
    write_file(dir / assets::kCapturedBootstrapName, c.text);
    const std::vector<double> sentinel{1.5, 2.5, 3.5};
    std::vector<double> qpos = sentinel;
    std::vector<double> qvel = sentinel;
    const assets::AssetStatus got =
        assets::load_captured_initial_state((dir / "model.xml").string(), c.nq, c.nv, qpos, qvel);
    require_status(got, c.want, ctx + "status");
    if (c.want == assets::AssetStatus::kOk) {
        require(qpos.size() == c.qpos_want.size(), ctx + "qpos size");
        require(qvel.size() == c.qvel_want.size(), ctx + "qvel size");
        for (std::size_t i = 0; i < qpos.size(); ++i) {
            require(same_bits(qpos[i], c.qpos_want[i]), ctx + "qpos[" + std::to_string(i) + "]");
        }
        for (std::size_t i = 0; i < qvel.size(); ++i) {
            require(same_bits(qvel[i], c.qvel_want[i]), ctx + "qvel[" + std::to_string(i) + "]");
        }
    } else {
        require(qpos == sentinel, ctx + "a failed parse leaves qpos untouched");
        require(qvel == sentinel, ctx + "a failed parse leaves qvel untouched");
    }
    std::cout << std::format("  ok  {:<28} -> {}\n", c.name, assets::asset_status_name(got));
}

void test_captured_errors(const fs::path &xml) {
    const std::string real = read_file(xml.parent_path() / assets::kCapturedBootstrapName);
    const std::vector<double> none;

    // Wrong dims on the real file: nq and nv mismatches.
    {
        const fs::path dir = fresh_dir("json_real_dims");
        write_file(dir / assets::kCapturedBootstrapName, real);
        std::vector<double> qpos{1.5, 2.5, 3.5};
        std::vector<double> qvel{1.5, 2.5, 3.5};
        const std::vector<double> sentinel{1.5, 2.5, 3.5};
        require_status(
            assets::load_captured_initial_state((dir / "model.xml").string(), 25, 26, qpos, qvel),
            assets::AssetStatus::kDimMismatch, "real bootstrap with nq = 25");
        require(qpos == sentinel && qvel == sentinel, "a dims mismatch leaves outputs untouched");
        require_status(
            assets::load_captured_initial_state((dir / "model.xml").string(), 26, 25, qpos, qvel),
            assets::AssetStatus::kDimMismatch, "real bootstrap with nv = 25");
        require(qpos == sentinel && qvel == sentinel, "a dims mismatch leaves outputs untouched");
    }

    // Truncated real file (mid-array and missing tail).
    {
        const fs::path dir = fresh_dir("json_real_truncated");
        write_file(dir / assets::kCapturedBootstrapName, real.substr(0, real.size() / 2));
        std::vector<double> qpos, qvel;
        require_status(
            assets::load_captured_initial_state((dir / "model.xml").string(), 26, 26, qpos, qvel),
            assets::AssetStatus::kParseError, "real bootstrap truncated mid-array");
        require(qpos.empty() && qvel.empty(), "a truncated parse leaves outputs empty");
        write_file(dir / assets::kCapturedBootstrapName, real.substr(0, real.size() - 2));
        require_status(
            assets::load_captured_initial_state((dir / "model.xml").string(), 26, 26, qpos, qvel),
            assets::AssetStatus::kParseError, "real bootstrap missing its tail");
        require(qpos.empty() && qvel.empty(), "a truncated parse leaves outputs empty");
    }

    // Missing file.
    {
        const fs::path dir = fresh_dir("json_missing");
        std::vector<double> qpos, qvel;
        require_status(
            assets::load_captured_initial_state((dir / "model.xml").string(), 26, 26, qpos, qvel),
            assets::AssetStatus::kMissingFile, "missing captured_bootstrap.json");
        require(qpos.empty() && qvel.empty(), "a missing file leaves outputs empty");
    }

    // Synthetic cases: positive controls and malformed inputs.
    const std::string ok_nq2 =
        R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1.0, 2.0, 3.0, 4.0]}})";
    const std::vector<JsonCase> json_cases = {
        {.name = "valid_minimal", .text = ok_nq2, .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kOk, .qpos_want = {1.0, 2.0}, .qvel_want = {3.0, 4.0}},
        {.name = "extra_trailing_values",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4, 5]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kOk, .qpos_want = {1.0, 2.0}, .qvel_want = {3.0, 4.0}},
        {.name = "key_text_inside_string",
         .text = R"({"note": "\"state\": {", "state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kOk, .qpos_want = {1.0, 2.0}, .qvel_want = {3.0, 4.0}},
        {.name = "nq_outside_model_dims_ignored",
         .text = R"({"nq": 99, "state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kOk, .qpos_want = {1.0, 2.0}, .qvel_want = {3.0, 4.0}},
        {.name = "nq_only_outside_model_dims",
         .text = R"({"nq": 2, "nv": 2, "state": {"model_dims": {"x": 1}, "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "dims_mismatch",
         .text = R"({"state": {"model_dims": {"nq": 3, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4, 5]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kDimMismatch, .qpos_want = none, .qvel_want = none},
        {.name = "bad_number_1.2.3",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1.2.3, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "leading_zero_01",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 01, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "nan_literal",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, NaN, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "overflow_inf",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1e400, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "string_element",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, "1.0", 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "trailing_comma_array",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4,]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "trailing_comma_object",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2,}, "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "short_integration_state",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "empty_integration_state",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": []}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "float_nq",
         .text = R"({"state": {"model_dims": {"nq": 2.0, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "duplicate_integration_state",
         .text = R"({"state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4], "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "missing_state",
         .text = R"({"other": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "state_not_object", .text = R"({"state": [1, 2]})", .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "trailing_garbage", .text = ok_nq2 + " x", .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "empty_file", .text = "", .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "raw_control_char_in_string",
         .text = std::string("{\"k\": \"a\x01\", \"state\": {\"model_dims\": {\"nq\": 2, \"nv\": 2}, \"integration_state\": [0.5, 1, 2, 3, 4]}}"),
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
        {.name = "nesting_too_deep",
         .text = R"({"x": )" + std::string(100, '[') + std::string(100, ']') +
             R"(, "state": {"model_dims": {"nq": 2, "nv": 2}, "integration_state": [0.5, 1, 2, 3, 4]}})",
         .nq = 2, .nv = 2,
         .want = assets::AssetStatus::kParseError, .qpos_want = none, .qvel_want = none},
    };
    int index = 0;
    for (const JsonCase &c : json_cases) {
        run_json_case(c, index);
        ++index;
    }
}

constexpr std::array<TestCase, 6> cases{{
    {.name = "status_names", .run = test_status_names},
    {.name = "hfield_bin_real", .run = test_hfield_bin_real},
    {.name = "extract_real", .run = test_extract_real},
    {.name = "captured_real", .run = test_captured_real},
    {.name = "hfield_errors", .run = test_hfield_errors},
    {.name = "captured_errors", .run = test_captured_errors},
}};

int run_case(const TestCase &test_case, const fs::path &xml) {
    try {
        test_case.run(xml);
        std::cout << "PASS " << test_case.name << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
        return 1;
    }
}
} // namespace

int main(int argc, char **argv) {
    try {
        const std::span<char *> arguments = bike::platform::main_arguments(argc, argv);
        if (arguments.size() == 2 && std::string_view(arguments[1]) == "--list") {
            for (const TestCase &test_case : cases) {
                std::cout << test_case.name << '\n';
            }
            return 0;
        }
        if (arguments.size() > 2) {
            std::cerr << "usage: bike_test_gpu_model_assets [case-name|--list]\n";
            return 2;
        }
        const std::string_view requested =
            arguments.size() == 1 ? "all" : std::string_view(arguments[1]);

        // An unknown case name is a usage error (exit 2) reported before any
        // model I/O, mirroring test_gpu_model_geometry.cpp.
        bool found = requested == "all";
        for (const TestCase &test_case : cases) {
            found = found || requested == test_case.name;
        }
        if (!found) {
            std::cerr << "unknown gpu model_assets case: " << requested << '\n';
            return 2;
        }

        const fs::path xml = find_probe_model_xml();
        if (xml.empty()) {
            std::cerr << "FAIL: " << kRelModelPath
                      << " not found (run from the repo root)\n";
            return 1;
        }
        int status = 0;
        for (const TestCase &test_case : cases) {
            if (requested == "all" || requested == test_case.name) {
                if (run_case(test_case, xml) != 0) {
                    status = 1;
                }
            }
        }
        return status;
    } catch (const std::exception &error) {
        // A throwing argument envelope is a harness failure, not a usage error.
        std::cerr << "FAIL argument handling: " << error.what() << '\n';
        return 1;
    }
}
