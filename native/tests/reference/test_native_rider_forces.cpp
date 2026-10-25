// Completion 3.14b (T52). Source: tests/reference/test_native_rider_forces.py.
// The rider_forces writer's bitwise sweep plus the P2 accumulator-order
// gate, replayed against the platform goldens. The canonical episode never
// engages 'seated_interfaces', so the sweep replays the seated-variant
// plant: the C++ side rebuilds it through model::generate_mjcf with the
// seated RiderSpecs and feeds the frozen per-path descriptors through the
// projected 'rider_forces' section.

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "config_types.hpp"
#include "configuration/rider_spec.hpp"
#include "diag.hpp"
#include "engine_call.hpp"
#include "support/golden.hpp"
#include "golden/episode.hpp"
#include "io/float_format.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "support/migration_test.hpp"
#include "model/builder.hpp"
#include "research/environment.hpp"
#include "rider/attachment_wrench.hpp"
#include "runtime/project_config.hpp"
#include "stepper.hpp"
#include "support/temporary_directory.hpp"
#include "terrain/profile.hpp"
#include "writer_config.hpp"
#include "writers/resistance.hpp"
#include "writers/tire_common.hpp"

namespace {
namespace cfg = bike::configuration;
using nativecfg::WriterConfig;
namespace golden_ep = bike::golden;
namespace mdl = bike::model;
namespace trn = bike::terrain;

using bike::io::repr_binary64;

constexpr std::string_view kFile =
    "tests/reference/test_native_rider_forces.py::";

std::filesystem::path repo_file(std::string_view relative) {
    // tests/fixtures/golden -> repo root is three levels up.
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

const std::string &bench_mjb() {
    static const std::string path =
        repo_file("tools/proto_native_bench/artifacts/model.mjb").string();
    return path;
}

void scalar(bt::oracle::Node &golden, double actual, std::string_view label) {
    const std::array<double, 1> value{actual};
    golden.f64(value, std::span<const std::size_t>{}, label);
}

std::string decode_f64_text(const std::vector<double> &encoded) {
    std::string out;
    out.reserve(encoded.size());
    for (const double code : encoded)
        out.push_back(static_cast<char>(static_cast<unsigned char>(code)));
    return out;
}

// The artifact arrays decode once through NpyArray::doubles (NaN-tolerant:
// tire_state carries NaN for unset brush segments); rows then slice the
// flat C-order buffer.
std::vector<double> row_of(const std::vector<double> &flat, std::size_t row,
                           std::size_t width) {
    bt::require(row * width + width <= flat.size(),
                "episode row in bounds");
    const auto first =
        flat.begin() + static_cast<std::ptrdiff_t>(row * width);
    return {first, first + static_cast<std::ptrdiff_t>(width)};
}

// ---- test_rider_forces_qfrc_bitwise -------------------------------------

struct PathSpec {
    std::string name, joint;
    double stiffness, damping, preload, offset;
    bool unilateral;
    int qposadr, dofadr;
};

nativecfg::WriterConfig rf_config(const std::vector<PathSpec> &paths) {
    nativecfg::WriterConfig config;
    auto &section = config.rider_forces.emplace();
    for (const PathSpec &p : paths)
        section.paths.push_back({.name = p.name, .joint = p.joint,
                                 .stiffness_n_m = p.stiffness,
                                 .damping_ns_m = p.damping,
                                 .preload_deflection_m = p.preload,
                                 .offset_m = p.offset,
                                 .unilateral = p.unilateral});
    return config;
}

std::vector<std::byte> save_mjb(const mjModel *model) {
    int size = 0;
    engine::ErrorBuffer error;
    struct SizeCall {
        const mjModel *model;
        int size;
    } call{.model = model, .size = 0};
    if (!engine::invoke(
            [](void *raw) noexcept {
                auto &c = *static_cast<SizeCall *>(raw);
                c.size = static_cast<int>(mj_sizeModel(c.model));
            },
            &call, error))
        engine::throw_failure(error);
    size = call.size;
    std::vector<std::byte> image(static_cast<std::size_t>(size));
    struct SaveCall {
        const mjModel *model;
        void *bytes;
        int size;
    } save{.model = model, .bytes = image.data(), .size = size};
    if (!engine::invoke(
            [](void *raw) noexcept {
                const auto &c = *static_cast<SaveCall *>(raw);
                mj_saveModel(c.model, nullptr, c.bytes, c.size);
            },
            &save, error))
        engine::throw_failure(error);
    return image;
}

std::string f64_repr(double v) { return repr_binary64(v); }

std::string np_f64(double v) {
    // repr(np.float64(x)) under NumPy 2.x: the np.float64 wrapper spelling.
    return "np.float64(" + repr_binary64(v) + ")";
}

// BuildRequest embeds the full rider/track spec — ~9 KiB of automatic
// storage, over the Linux -Wframe-larger-than=8192 gate (main carries the
// same exemption below).
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void rider_forces_sweep(bt::oracle::Node &golden) {
    const std::vector<double> qpos0 = golden.f64_input("input.qpos0");
    const std::vector<double> dims = golden.f64_input("input.dims");
    // The seated-variant plant: BikeSpecs() + RiderSpecs(variant='seated')
    // through the same generate_mujoco_xml(mode='ride', rider=...) path.
    mdl::BuildRequest request;
    request.mode = "ride";
    cfg::RiderSpecs rider{};
    rider.variant = "seated";
    request.rider = rider;
    request.include_rider = true;
    bike::model::ModelOwner model =
        mdl::compile_mjcf(mdl::generate_mjcf(request));
    bt::require(dims.size() == 3 &&
                    static_cast<int>(dims[0]) == model->nq &&
                    static_cast<int>(dims[1]) == model->nv &&
                    static_cast<int>(dims[2]) == model->njnt,
                "seated model dims match the oracle plant");
    std::vector<PathSpec> paths;
    for (int i = 0;; ++i) {
        const std::string base = "input.path" + std::to_string(i);
        if (golden.peek_label() != base + ".name") break;
        PathSpec p;
        p.name = decode_f64_text(golden.f64_input(base + ".name"));
        p.joint = decode_f64_text(golden.f64_input(base + ".joint"));
        const std::vector<double> params =
            golden.f64_input(base + ".params");
        p.stiffness = params[0];
        p.damping = params[1];
        p.preload = params[2];
        p.offset = params[3];
        p.unilateral = params[4] != 0.;
        p.qposadr = static_cast<int>(params[5]);
        p.dofadr = static_cast<int>(params[6]);
        paths.push_back(std::move(p));
    }
    const int nv = static_cast<int>(model->nv);
    const int nq = static_cast<int>(model->nq);
    bt::require(static_cast<int>(qpos0.size()) == nq, "qpos0 width");
    // The rng state replays are frozen inputs; consume them BEFORE the
    // comparison stream like the source records.
    struct RngState {
        std::vector<double> qpos, qvel;
    };
    std::vector<RngState> rng;
    for (int j = 0;; ++j) {
        const std::string base =
            "input.rng" + std::to_string(j) + ".qpos";
        if (golden.peek_label() != base) break;
        rng.push_back({golden.f64_input(base),
                       golden.f64_input("input.rng" + std::to_string(j) +
                                        ".qvel")});
    }
    Stepper st(std::span<const std::byte>(save_mjb(model.get())),
               rf_config(paths));
    const std::vector<double> zeros_v(static_cast<std::size_t>(nv), 0.);
    // act = np.zeros(model.na) in the source (empty when na==0).
    const std::vector<double> zeros_a(
        static_cast<std::size_t>(model->na), 0.);
    auto compare = [&](const std::vector<double> &qpos,
                       const std::vector<double> &qvel,
                       const std::string &label) {
        st.set_state(qpos, qvel, zeros_a, zeros_v, 0.);
        golden.f64(st.rider_forces_qfrc(), "rider_forces " + label);
    };
    for (const PathSpec &p : paths) {
        const double bound = p.preload + p.offset;
        const std::array<std::pair<double, bool>, 8> qs{{
            {0.0, false}, {0.5 * bound, false},
            {std::nextafter(bound, -std::numeric_limits<double>::infinity()),
             true},
            {bound, false},
            {std::nextafter(bound, std::numeric_limits<double>::infinity()),
             true},
            {bound + 0.02, false}, {-0.03, false}, {0.05, false}}};
        const double thr = p.stiffness * bound / p.damping;
        const std::array<double, 6> qds{0., -0.5 * thr, 0.5 * thr,
                                        0.9 * thr, thr, 1.5 * thr};
        for (const auto &[q, is_np] : qs)
            for (const double qd : qds) {
                std::vector<double> qpos = qpos0;
                std::vector<double> qvel(static_cast<std::size_t>(nv), 0.);
                qpos[static_cast<std::size_t>(p.qposadr)] = q;
                qvel[static_cast<std::size_t>(p.dofadr)] = qd;
                compare(qpos, qvel,
                        p.name + " q=" +
                            (is_np ? np_f64(q) : f64_repr(q)) +
                            " qd=" + f64_repr(qd));
            }
    }
    for (const auto &[tag, shift] :
         std::array{std::pair{"boundary", 0.0},
                    std::pair{"gap", 0.03},
                    std::pair{"compressed", -0.02}}) {
        std::vector<double> qpos = qpos0;
        std::vector<double> qvel(static_cast<std::size_t>(nv), 0.);
        for (const PathSpec &p : paths) {
            qpos[static_cast<std::size_t>(p.qposadr)] =
                p.preload + p.offset + shift;
            qvel[static_cast<std::size_t>(p.dofadr)] = 0.37;
        }
        compare(qpos, qvel, std::string("all-") + tag);
    }
    for (std::size_t j = 0; j < rng.size(); ++j)
        compare(rng[j].qpos, rng[j].qvel, "rng" + std::to_string(j));
    golden.done();
}

void test_rider_forces_qfrc_bitwise() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    rider_forces_sweep(golden);
}

// ---- test_project_emits_resolved_paths ----------------------------------

void test_project_emits_resolved_paths() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const std::vector<double> dims = golden.f64_input("input.dims");
    // RideSimulation(): the default seated ride plant, projected through
    // the same tools.native_config.project seam.
    mdl::BuildRequest request;
    request.mode = "ride";
    cfg::RiderSpecs rider{};
    rider.variant = "seated";
    request.rider = rider;
    request.include_rider = true;
    bike::model::ModelOwner model =
        mdl::compile_mjcf(mdl::generate_mjcf(request));
    bt::require(dims.size() == 3 &&
                    static_cast<int>(dims[0]) == model->nq &&
                    static_cast<int>(dims[1]) == model->nv &&
                    static_cast<int>(dims[2]) == model->njnt,
                "RideSimulation model dims");
    const auto track = trn::flat();
    nativecfg::WriterConfig config = nativecfg::decode(
        runtime::project_writer_config(request, *model, track));
    bt::require(config.rider_forces.has_value(),
                "project emits rider_forces for the seated plant");
    // rf.set_pedal_offsets(0.023, -0.031) runs before project() in the
    // source: the live applier's paths carry the resolved offsets and
    // the projected section reads them back. project_writer_config emits
    // offset_m=0, so the same set_pedal_offsets rule patches them in.
    for (auto &p : config.rider_forces->paths) {
        if (p.name == "rider_leg_front") p.offset_m = 0.023;
        if (p.name == "rider_leg_rear") p.offset_m = -0.031;
    }
    const std::vector<nativecfg::RiderPathConfig> &paths =
        config.rider_forces->paths;
    for (int i = 0;; ++i) {
        const std::string base = "expected.path" + std::to_string(i);
        if (golden.peek_label() != base + ".name") break;
        const std::string name =
            decode_f64_text(golden.f64_input(base + ".name"));
        const std::string joint =
            decode_f64_text(golden.f64_input(base + ".joint"));
        const std::vector<double> expected =
            golden.f64_input(base);
        bt::require(static_cast<std::size_t>(i) < paths.size(),
                    "projected path count");
        const nativecfg::RiderPathConfig &got =
            paths[static_cast<std::size_t>(i)];
        bt::require(got.name == name, "path name " + std::to_string(i));
        bt::require(got.joint == joint, "path joint " + std::to_string(i));
        const int jid = mj_name2id(model.get(), mjOBJ_JOINT,
                                   got.joint.c_str());
        bt::require(jid >= 0, "projected joint resolves");
        const std::vector<double> actual{
            got.stiffness_n_m, got.damping_ns_m, got.preload_deflection_m,
            got.offset_m, got.unilateral ? 1. : 0.,
            static_cast<double>(std::views::counted(
                model->jnt_qposadr, model->njnt)
                    [static_cast<std::size_t>(jid)])};
        bt::require(actual.size() == expected.size() &&
                        std::ranges::equal(
                            actual, expected,
                            [](double a, double e) {
                                return std::bit_cast<std::uint64_t>(a) ==
                                       std::bit_cast<std::uint64_t>(e);
                            }),
                    "projected path fields bitwise " + std::to_string(i));
    }
    const auto expected_nv = golden.f64_input("expected.nv");
    bt::require(expected_nv.size() == 1 &&
                    static_cast<int>(expected_nv[0]) == model->nv,
                "expected.nv matches the compiled model");
    // The projected section must drive the native writer.
    Stepper st(std::span<const std::byte>(save_mjb(model.get())), config);
    bt::require(st.rider_forces_qfrc().size() ==
                    static_cast<std::size_t>(model->nv),
                "projected section drives rider_forces_qfrc");
    golden.done();
}

// ---- hand-built / validation tests ---------------------------------------

nativecfg::WriterConfig rf_cfg(
    const std::vector<std::pair<std::string, bike::io::Value>> &overrides = {},
    const std::vector<std::string> &drop = {}) {
    // _rf_cfg: one 'probe_path' on the 'root_x' unlimited slide.
    std::map<std::string, bike::io::Value> path{
        {"name", bike::io::Value{std::string("probe_path")}},
        {"joint", bike::io::Value{std::string("root_x")}},
        {"stiffness_n_m", bike::io::Value{bike::io::Number{"1e5"}}},
        {"damping_ns_m", bike::io::Value{bike::io::Number{"1e3"}}},
        {"preload_deflection_m",
         bike::io::Value{bike::io::Number{"0.01"}}},
        {"unilateral", bike::io::Value{true}},
        {"offset_m", bike::io::Value{bike::io::Number{"0.0"}}}};
    for (const auto &[key, value] : overrides) path[key] = value;
    for (const std::string &key : drop) path.erase(key);
    bike::io::Value::Object obj;
    for (auto &[key, value] : path) obj.emplace_back(key, value);
    return nativecfg::decode(bike::io::Value{bike::io::Value::Object{
        {"schema", bike::io::Value{bike::io::Number{"1"}}},
        {"rider_forces", bike::io::Value{bike::io::Value::Object{
                             {"paths", bike::io::Value{bike::io::Value::Array{
                                           bike::io::Value{std::move(obj)}}}}}}}}});
}

void test_rider_forces_hand_built_smoke() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Stepper st(bench_mjb(), rf_cfg());
    const std::vector<double> got = st.rider_forces_qfrc();
    bt::require(got.size() == static_cast<std::size_t>(st.model()->nv),
                "hand-built qfrc is nv wide");
    // q=qd=0: force = max(0, 1e5*(0.01+0-0) - 1e3*0) — same expression order.
    scalar(golden, got[0], "hand-built path force");
    bt::require(std::bit_cast<std::uint64_t>(got[0]) ==
                    std::bit_cast<std::uint64_t>(
                        std::max(0., 1e5 * (0.01 + 0. - 0.) - 1e3 * 0.)),
                "hand-built path force value");
    golden.done();
}

template <class Action> void throws(Action &&action, std::string_view needle) {
    try {
        std::forward<Action>(action)();
    } catch (const std::exception &error) {
        bt::require(std::string(error.what()).find(needle) !=
                        std::string::npos,
                    std::string("error names '") + std::string(needle) +
                        "': " + error.what());
        return;
    }
    bt::require(false, "expected a rejection naming " + std::string(needle));
}

void test_rider_forces_requires_config() {
    Stepper bare(bench_mjb());
    throws([&] { static_cast<void>(bare.rider_forces_qfrc()); },
           "rider_forces");
    Stepper empty(bench_mjb(), nativecfg::WriterConfig{});
    throws([&] { static_cast<void>(empty.rider_forces_qfrc()); },
           "rider_forces");
}

void test_rider_forces_missing_key_names_it() {
    // Source: bike_native.Stepper(MJB, cfg) — the decode failure happens
    // inside construction, so wrap it the same way.
    throws(
        [] {
            Stepper st(bench_mjb(), nativecfg::decode(bike::io::parse_json(
                                        R"({"schema":1,"rider_forces":{}})")));
        },
        "paths");
    for (const char *key : {"joint", "stiffness_n_m", "damping_ns_m",
                            "preload_deflection_m", "unilateral",
                            "offset_m"})
        throws([&] { Stepper st(bench_mjb(), rf_cfg({}, {key})); }, key);
    throws(
        [] {
            Stepper st(
                bench_mjb(),
                nativecfg::decode(bike::io::parse_json(
                    R"({"schema":1,"rider_forces":{"paths":{"joint":"root_x"}}})")));
        },
        "paths");
}

void test_rider_forces_joint_validation() {
    for (const auto &[joint, needle] :
         std::array{std::pair{"no_such_joint", "no joint"},
                    std::pair{"front_wheel_spin", "unlimited slide"},
                    std::pair{"shock_stroke", "unlimited slide"}})
        throws(
            [&] {
                Stepper st(bench_mjb(), rf_cfg(
                                            {{"joint", bike::io::Value{
                                                            std::string(joint)}}}));
            },
            needle);
}

// ---- accumulator tests -----------------------------------------------------

// Stepper::total takes a span of owning vectors: stage the ordered
// components the Python dict's insertion order implies, then fold.
std::vector<double> total_of(Stepper &st,
                             std::initializer_list<std::vector<double>> comps) {
    const std::vector<std::vector<double>> owned(comps.begin(), comps.end());
    return st.total(owned);
}

void test_accumulator_order_bitwise() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    Stepper st(bench_mjb());
    const std::size_t nv = static_cast<std::size_t>(st.model()->nv);
    const std::vector<double> a(nv, 1e20), b(nv, 1.), c(nv, -1e20);
    // (a,b,c) folds to 0 while (a,c,b) folds to 1 — order must matter.
    const std::vector<double> got1 = total_of(st, {a, b, c});
    const std::vector<double> got2 = total_of(st, {a, c, b});
    golden.f64(got1, "order a,b,c");
    golden.f64(got2, "order a,c,b");
    bt::require(got1 != got2, "the order actually mattered");
    golden.f64(got1, "absorbed fold");
    for (int trial = 0; trial < 6; ++trial) {
        const std::string base =
            "input.t" + std::to_string(trial);
        const auto count =
            static_cast<std::size_t>(golden.f64_input(base + ".count")[0]);
        std::vector<std::vector<double>> comps;
        for (std::size_t i = 0; i < count; ++i)
            comps.push_back(
                golden.f64_input(base + ".c" + std::to_string(i)));
        for (const bool reversed : {false, true}) {
            std::vector<std::vector<double>> order = comps;
            if (reversed) std::ranges::reverse(order);
            golden.f64(st.total(order),
                       "fuzz trial " + std::to_string(trial));
        }
    }
    golden.f64(total_of(st, {}), "empty total");
    golden.done();
}

void test_accumulator_total_validates() {
    Stepper st(bench_mjb());
    const std::size_t nv = static_cast<std::size_t>(st.model()->nv);
    const std::vector<double> bad_width(nv + 1, 0.);
    const std::vector<double> ok(nv, 0.);
    // The typed face names width/nonfinite violations with acc.add's
    // 'invalid generalized force'; the binding's 'total.<key>' spelling
    // is dict-surface only. The rank-2 and non-array cases are likewise
    // binding-cast rejections with no typed-face counterpart (documented
    // drops in the port map).
    throws([&] { static_cast<void>(total_of(st, {bad_width})); },
           "invalid generalized force");
    throws([&] { static_cast<void>(total_of(st, {ok, bad_width})); },
           "invalid generalized force");
    throws(
        [&] {
            static_cast<void>(total_of(
                st, {std::vector<double>(
                        nv, std::numeric_limits<double>::infinity())}));
        },
        "invalid generalized force");
    throws(
        [&] {
            static_cast<void>(total_of(
                st, {std::vector<double>(
                        nv, std::numeric_limits<double>::quiet_NaN())}));
        },
        "invalid generalized force");
}

void conversion_case(const char *dtype, const char *layout) {
    const std::string node = std::string(kFile) +
        "test_accumulator_total_accepts_array_conversion[" + dtype + "-" +
        layout + "]";
    bt::oracle::Node golden(node);
    Stepper st(bench_mjb());
    const std::size_t nv = static_cast<std::size_t>(st.model()->nv);
    std::vector<double> values(nv);
    for (std::size_t i = 0; i < nv; ++i) {
        const std::size_t index =
            std::string_view(layout) == "reversed" ? nv - 1 - i : i;
        values[i] = static_cast<double>(index);
    }
    const std::vector<double> ones(nv, 1.);
    const std::string label = std::string("total converts <class 'numpy.") +
                              dtype + std::string("'> ") + layout;
    golden.f64(total_of(st, {values, ones}), label);
    golden.done();
}

void test_accumulator_total_accepts_array_conversion() {
    for (const char *dtype : {"float32", "float64", "int32"})
        for (const char *layout : {"contiguous", "strided", "reversed"})
            conversion_case(dtype, layout);
}

// ---- test_accumulator_order_golden ----------------------------------------

struct OwnedSide {
    std::vector<double> loads;
    // TireSideInput::patch_working is a span<const bool>: vector<bool>'s
    // packed storage cannot back a bool span, so own a raw bool block.
    std::unique_ptr<bool[]> working;
    std::size_t working_n = 0;
    double radius = 0.;
    [[nodiscard]] TireSideInput view() const {
        return {.patch_loads = loads,
                .patch_working = std::views::counted(
                    working.get(), static_cast<std::ptrdiff_t>(working_n)),
                .effective_radius_m = radius};
    }
};

OwnedSide side_input(const TireSnapshot &snapshot) {
    OwnedSide out;
    out.radius = snapshot.effective_radius_m;
    out.loads.reserve(snapshot.patches.size());
    const std::size_t n = snapshot.patches.size();
    out.working = std::make_unique<bool[]>(n);
    out.working_n = n;
    for (std::size_t i = 0; i < n; ++i) {
        const TirePatch &p = snapshot.patches[i];
        out.loads.push_back(p.normal_load_n);
        std::views::counted(out.working.get(),
            static_cast<std::ptrdiff_t>(n))[i] = p.working_surface;
    }
    return out;
}

void test_accumulator_order_golden() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    // Platform-captured oracle artifact (freeze path): shared inputs
    // (request.toml) live under root/episode/, the npz under
    // root/<platform>/episode/.
    const auto directory = bt::oracle::root() /
        bt::oracle::platform() / "episode/pinned_savage_40";
    const golden_ep::EpisodeArtifact ep =
        golden_ep::load_episode(directory);
    const auto request = bike::research::read_research_request(
        bt::oracle::root() / "episode/pinned_savage_40/request.toml");
    bike::model::ModelOwner model =
        mdl::compile_mjcf(mdl::generate_mjcf(request.setup.model));
    const nativecfg::WriterConfig config = nativecfg::decode(
        runtime::project_writer_config(request.setup.model, *model,
                                             request.setup.track));
    Stepper st(std::span<const std::byte>(ep.model_image), config);
    // shapes: forces [K,F,nv], qfrc_applied [K,nv]
    const std::size_t k_steps = ep.qfrc_applied.shape.at(0);
    const std::size_t nv = ep.qfrc_applied.shape.at(1);
    const std::size_t f_names = ep.force_names.size();
    bt::require(ep.forces.shape.size() == 3 &&
                    ep.forces.shape[0] == k_steps &&
                    ep.forces.shape[1] == f_names &&
                    ep.forces.shape[2] == nv,
                "acc matrix shape matches qfrc_applied");
    bt::require(std::ranges::find(ep.force_names, "seated_interfaces") ==
                    ep.force_names.end(),
                "canonical episode never records seated_interfaces");
    bt::require(!config.rider_forces.has_value(),
                "canonical projection never emits rider_forces");
    const auto &manifest_obj = bike::io::as_object(ep.manifest, "manifest");
    const bike::io::Value *dt_member =
        bike::io::find_member(manifest_obj, "dt_s");
    bt::require(dt_member != nullptr, "manifest carries dt_s");
    const double dt = bike::io::as_double(*dt_member, "manifest dt_s");
    const std::vector<std::string> tire_names = ep.tire_state_names;
    const std::size_t tire_width = ep.tire_state.shape.at(1);
    const std::vector<double> qpos_flat = ep.state_qpos.doubles(true);
    const std::vector<double> qvel_flat = ep.state_qvel.doubles(true);
    const std::vector<double> act_flat = ep.state_act.doubles(true);
    const std::vector<double> warm_flat =
        ep.state_warmstart.doubles(true);
    const std::vector<double> time_flat = ep.state_time.doubles(true);
    const std::vector<double> tire_flat = ep.tire_state.doubles(true);
    const std::vector<double> forces_flat = ep.forces.doubles(true);
    for (std::size_t k = 0; k < k_steps; ++k) {
        st.set_state(row_of(qpos_flat, k,
                            static_cast<std::size_t>(model->nq)),
                     row_of(qvel_flat, k,
                            static_cast<std::size_t>(model->nv)),
                     row_of(act_flat, k,
                            static_cast<std::size_t>(model->na)),
                     row_of(warm_flat, k,
                            static_cast<std::size_t>(model->nv)),
                     row_of(time_flat, k, 1)[0]);
        st.forward();
        st.set_tire_state(tire_names,
                          row_of(tire_flat, k, tire_width));
        std::map<std::string, std::vector<double>> ported;
        for (auto &[name, qfrc] : st.suspension_components())
            ported.emplace(name, std::move(qfrc));
        ported.emplace("tires", st.tire_qfrc(dt));
        const auto snaps = st.tire_writer().snapshots();
        bt::require(snaps[0].has_value() && snaps[1].has_value(),
                    "canonical episode engages both tires");
        const OwnedSide front = side_input(*snaps[0]);
        const OwnedSide rear = side_input(*snaps[1]);
        for (auto &[name, qfrc] :
             st.resistance_components(front.view(), rear.view()))
            ported.emplace(name, std::move(qfrc));
        std::vector<std::vector<double>> components;
        components.reserve(f_names);
        for (std::size_t i = 0; i < f_names; ++i) {
            const auto found = ported.find(ep.force_names[i]);
            if (found != ported.end())
                components.push_back(found->second);
            else
                components.push_back(
                    row_of(forces_flat, k * f_names + i, nv));
        }
        golden.f64(st.total(components),
                   "acc.total step " + std::to_string(k));
    }
    golden.done();
}
} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_rider_forces_qfrc_bitwise();
        test_project_emits_resolved_paths();
        test_rider_forces_hand_built_smoke();
        test_rider_forces_requires_config();
        test_rider_forces_missing_key_names_it();
        test_rider_forces_joint_validation();
        test_accumulator_order_bitwise();
        test_accumulator_total_validates();
        test_accumulator_total_accepts_array_conversion();
        test_accumulator_order_golden();
    });
}
