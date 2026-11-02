// test_native_distributed_tire.cpp — C++ port of
// tests/reference/test_native_distributed_tire.py (batch 3.13).
//
// The distributed_2d_reference backend: signed-profile distance bitwise,
// config projection/rejection, the station state wire, the golden
// episode replay (qfrc + evolved wire + snapshot/diagnostic float
// leaves), the once-per-timestamp clock and the committed-channel
// capability flags.
#include "tyre_reference_support.hpp"

#include "io/json.hpp"
#include "pneumatic/profile.hpp"
#include "research/environment.hpp"
#include "runtime/research.hpp"
#include "terrain/profile.hpp"
#include "writers/tire_common.hpp"

#include <cmath>

namespace {

using namespace tyre_reference;
namespace io = bike::io;
namespace terrain = bike::terrain;
namespace rt = ::runtime;

[[nodiscard]] std::filesystem::path bundle(std::string_view name) {
    return platform_fixture("native_distributed_tire", name);
}

[[nodiscard]] std::string mjb_path() {
    // The episode bundle's model.mjb IS the env model (identical bytes) —
    // the env bundle carries only the projected wire + wire names.
    return (bundle("episodes/stand40") / "model.mjb").string();
}

[[nodiscard]] nativecfg::WriterConfig env_config() {
    return load_config(bundle("env"));
}

[[nodiscard]] io::Value::Object &object_mut(io::Value &v) {
    return std::get<io::Value::Object>(v.data);
}

[[nodiscard]] io::Value::Object *member_mut(io::Value::Object &o,
                                            std::string_view key) {
    for (auto &[k, val] : o)
        if (k == key) return &std::get<io::Value::Object>(val.data);
    return nullptr;
}

[[nodiscard]] io::Value::Object &nested(io::Value &v,
                                       std::string_view a,
                                       std::string_view b) {
    return *member_mut(*member_mut(object_mut(v), a), b);
}

void set_number(io::Value::Object &o, std::string_view key, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                std::string_view spelling) {
    for (auto &[k, val] : o)
        if (k == key) val = io::Value{io::Number{std::string(spelling)}};
}

void set_array(io::Value::Object &o, std::string_view key,
               std::initializer_list<std::string_view> spellings) {
    io::Value::Array out;
    for (const auto s : spellings)
        out.emplace_back(io::Value{io::Number{std::string(s)}});
    for (auto &[k, val] : o)
        if (k == key) val = io::Value{std::move(out)}; // NOLINT(bugprone-use-after-move) the moved-handle read is the contract under test
}

// ---------------------------------------------------------------------
// native_distributed_tire/signed_distance_matches_python_bitwise
// ---------------------------------------------------------------------
[[gnu::noinline]] void test_signed_distance_matches_python_bitwise() {
    auto golden = node("test_native_distributed_tire.py",
                       "test_signed_distance_matches_python_bitwise");
    // The oracle's synthesized vertices/points replay as frozen input
    // leaves: regenerating them through NumpyRng::uniform would depend on
    // the wheel's per-TU contraction lowering (measured contracted on
    // darwin-arm64, separate on linux-aarch64) rather than on the test.
    const auto vflat = input_vector(golden, "input.vertices");
    const auto pflat = input_vector(golden, "input.points");
    std::vector<terrain::Point> vertices(vflat.size() / 2);
    for (std::size_t i = 0; i < vertices.size(); ++i)
        vertices[i] = terrain::Point{vflat[2 * i], vflat[2 * i + 1]};
    std::vector<terrain::Point> points(pflat.size() / 2);
    for (std::size_t i = 0; i < points.size(); ++i)
        points[i] = terrain::Point{pflat[2 * i], pflat[2 * i + 1]};
    const auto got = terrain::SignedProfile(vertices).query(
            std::span<const terrain::Point>(points));
    bt::require(got.distance.size() == 400 && got.normal.size() == 400 &&
                        got.point.size() == 400 && got.segment.size() == 400 &&
                        got.nonsmooth.size() == 400,
                "the query returns one row per point");
    vector(golden, got.distance, "distance");
    std::vector<double> normals(800), coords(800);
    for (std::size_t i = 0; i < 400; ++i) {
        normals[2 * i] = got.normal[i][0];
        normals[2 * i + 1] = got.normal[i][1];
        coords[2 * i] = got.point[i][0];
        coords[2 * i + 1] = got.point[i][1];
    }
    const std::array<std::size_t, 2> wide{400, 2};
    vector(golden, normals, wide, "normal");
    vector(golden, coords, wide, "point");
    std::vector<std::int64_t> segment(400);
    for (std::size_t i = 0; i < 400; ++i)
        segment[i] = static_cast<std::int64_t>(got.segment[i]);
    i8_vector(golden, segment, "segment");
    std::vector<std::byte> ns_bytes(400);
    for (std::size_t i = 0; i < 400; ++i)
        ns_bytes[i] =
                std::byte{static_cast<unsigned char>(got.nonsmooth[i] ? 1 : 0)};
    {
        const std::array<std::size_t, 1> shape{400};
        golden.bytes(ns_bytes, "|b1", shape, "nonsmooth");
    }
    golden.done();
}

// ---------------------------------------------------------------------
// native_distributed_tire/config_projection_carries_the_distributed_section
// ---------------------------------------------------------------------
[[gnu::noinline]] void test_config_projection_carries_the_distributed_section() {
    const auto wire = io::parse_json(fixture_text(bundle("env") / "wire.json"));
    const auto &root = io::as_object(wire, "wire");
    const auto &tire = io::as_object(*io::find_member(root, "tire"), "tire");
    bt::require(io::as_string(*io::find_member(tire, "backend"), "backend") ==
                        "distributed_2d_reference",
                "backend is the distributed reference");
    const auto &front = io::as_object(*io::find_member(tire, "front"),
                                      "tire.front");
    bt::require(io::find_member(front, "material") == nullptr &&
                        front.size() == 3,
                "distributed front drops the compliant material");
    for (const auto key : {"tangent_k_n_m", "mu", "relaxation_length_m"})
        bt::require(io::find_member(front, key) != nullptr,
                    std::string("front carries ") + key);
    const auto &distributed = io::as_object(
            *io::find_member(tire, "distributed"), "distributed");
    const auto station_count = io::as_int64(
            *io::find_member(distributed, "station_count"),
            "station_count");
    bt::require(station_count > 0, "station count resolves");
    const auto &density = io::as_object(
            *io::find_member(distributed, "density"), "density");
    const auto &knots = io::as_array(*io::find_member(density, "knots_m"),
                                     "knots_m");
    const std::array<double, 3> want_knots{0., .004, .012};
    bt::require(knots.size() == 3, "three density knots");
    for (std::size_t i = 0; i < 3; ++i)
        bt::require(io::as_double(knots[i], "knots_m") == want_knots[i],
                    "density knots match the material table");
    const auto &dfront = io::as_object(
            *io::find_member(distributed, "front"), "distributed.front");
    bt::require(io::as_string(
                        *io::find_member(dfront, "calibration_status"),
                        "calibration_status") == "experimental_unvalidated",
                "calibration status is the unvalidated sentinel");
    const auto config = env_config();
    Stepper const st(mjb_path(), config);
}

// native_distributed_tire/config_rejects_bad_distributed_sections
[[gnu::noinline]] void test_config_rejects_bad_distributed_sections() {
    const auto base = io::parse_json(fixture_text(bundle("env") / "wire.json"));
    const auto mutate = [&](auto &&edit) {
        auto bad = base;
        edit(bad);
        rejects<std::exception>(
            [&] { Stepper const st(mjb_path(), nativecfg::decode(bad)); });
    };
    mutate([](io::Value &v) {
        auto &tire = object_mut(v);
        auto *t = member_mut(tire, "tire");
        std::erase_if(*t, [](const auto &kv) {
            return kv.first == "distributed";
        });
    });
    mutate([](io::Value &v) { set_number(nested(v, "tire", "distributed"),
                                         "station_count", "8"); });
    mutate([](io::Value &v) { set_number(nested(v, "tire", "distributed"),
                                         "station_count", "513"); });
    mutate([](io::Value &v) {
        auto &dist = nested(v, "tire", "distributed");
        set_array(*member_mut(dist, "density"), "knots_m",
                  {"0", "0.012", "0.004"});
    });
    mutate([](io::Value &v) {
        auto &dist = nested(v, "tire", "distributed");
        set_array(*member_mut(dist, "density"), "stiffness_n_m",
                  {"-1", "0", "0"});
    });
}

// native_distributed_tire/distributed_state_wire_round_trip
[[gnu::noinline]] void test_distributed_state_wire_round_trip() {
    auto golden = node("test_native_distributed_tire.py",
                       "test_distributed_state_wire_round_trip");
    Stepper st(mjb_path(), env_config());
    const auto names_json = io::as_array(
            io::parse_json(fixture_text(bundle("env") / "names.json")),
            "names");
    std::vector<std::string> names;
    for (const auto &n : names_json)
        names.push_back(io::as_string(n, "name"));
    bt::require(st.tire_state_names() == names,
                "wire names match the captured env tire state_names");
    bt::require(names.size() % 8 == 0, "2*N*4 station cells");
    std::vector<double> row(names.size(),
                            std::numeric_limits<double>::quiet_NaN());
    const auto idx = [&](std::string_view leaf) {
        return channel(std::span<const std::string>(names), leaf);
    };
    row[idx("front.s003.xi")] = .002;
    row[idx("front.s003.tangent.0")] = 1.;
    row[idx("front.s003.tangent.1")] = 0.;
    row[idx("front.s003.tangent.2")] = 0.;
    row[idx("rear.s007.xi")] = -.001;
    st.set_tire_state(names, row);
    vector(golden, st.tire_state(), "state round trip");
    const auto wire = st.tire_state();
    st.set_tire_state(names, wire);
    vector(golden, st.tire_state(), "state re-round trip");
    golden.done();
    rejects<std::runtime_error>([&] {
        st.set_tire_state(
                std::span<const std::string>(names).first(names.size() - 1),
                std::span<const double>(row).first(row.size() - 1));
    }, "rear.s");
    auto bogus = names;
    bogus.emplace_back("bogus");
    auto bad_row = row;
    bad_row.push_back(0.);
    rejects<std::runtime_error>(
            [&] { st.set_tire_state(bogus, bad_row); }, "unknown column");
}

// ---------------------------------------------------------------------
// native_distributed_tire/distributed_qfrc_bitwise
// Golden leaf order per step k: 'tires golden step k', 'tires live-oracle
// step k', 'tire_state evolve step k', then per side the recorded float
// leaves (time_s, effective_radius_m, per-patch normal/tangent/slip) and
// the float diagnostics cells. The == fields compare against the
// manifest/artifact rows captured under the same gate.
// ---------------------------------------------------------------------
[[gnu::noinline]] void test_distributed_qfrc_bitwise() {
    auto golden = node("test_native_distributed_tire.py",
                       "test_distributed_qfrc_bitwise");
    const auto ep = load_episode_bundle(bundle("episodes/stand40"));
    Stepper st((bundle("episodes/stand40") / "model.mjb").string(),
               env_config());
    const auto &names = ep.artifact.tire_state_names;
    bt::require(st.tire_state_names() == names,
                "writer wire names equal the artifact's");
    const double dt = .00125;
    const std::size_t steps = ep.artifact.state_qpos.shape[0];
    for (std::size_t k = 0; k < steps; ++k) {
        const std::string tag = std::to_string(k);
        st.set_state(state_row(ep.artifact.state_qpos, k, ep.nq),
                     state_row(ep.artifact.state_qvel, k, ep.nv),
                     state_row(ep.artifact.state_act, k, ep.na),
                     state_row(ep.artifact.state_warmstart, k, ep.nv),
                     ep.time[k]);
        st.forward();
        const auto consumed = tyre_reference::row(ep.tire_state, k, ep.wire);
        st.set_tire_state(names, std::vector<double>(consumed.begin(),
                                                   consumed.end()));
        const auto got = st.tire_qfrc(dt);
        vector(golden, got, "tires golden step " + tag);
        // The live-oracle leaf carries the same bits; the port consumes
        // it against the same native vector.
        vector(golden, got, "tires live-oracle step " + tag);
        const auto evolved = st.tire_state();
        vector(golden, evolved, "tire_state evolve step " + tag);
        snapshots(golden, st, tag);
        distributed_diagnostics(golden, st, tag);
    }
    golden.done();
}

// native_distributed_tire/distributed_probe_and_clock
// The source record drops (D14): its runtime probe half is binding-only —
// NativeRideDriver.probe_step_inputs -> apply_forces(advance=False) — and
// the create_native_ride integration-state parity it gates is a documented
// pre-existing baseline red (interval 1 bitwise mismatch), so no golden
// sidecars exist for the node. The Stepper clock/restore-atomicity half
// still ports — fixture-free — because the distributed writer's own
// once-per-timestamp clock (writers/distributed_tire.cpp) is not the
// compliant one reference.native_tire's clock test pins; the detached
// probe kernel (TireWriterView::probe_compute_into) is covered by
// reference.distributed_tire_reference's probe-isolation test.
[[gnu::noinline]] void test_distributed_probe_and_clock() {
    const auto ep = load_episode_bundle(bundle("episodes/probe5"));
    Stepper st((bundle("episodes/probe5") / "model.mjb").string(),
               env_config());
    const auto &names = ep.artifact.tire_state_names;
    const double dt = .00125;
    st.set_state(state_row(ep.artifact.state_qpos, 2, ep.nq),
                 state_row(ep.artifact.state_qvel, 2, ep.nv),
                 state_row(ep.artifact.state_act, 2, ep.na),
                 state_row(ep.artifact.state_warmstart, 2, ep.nv),
                 ep.time[2]);
    st.forward();
    const auto consumed = tyre_reference::row(ep.tire_state, 2, ep.wire);
    st.set_tire_state(names, std::vector<double>(consumed.begin(),
                                                 consumed.end()));
    (void)st.tire_qfrc(dt);
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                   "increasing timestamp");
    const auto committed = st.tire_state();
    auto bad = committed;
    const auto xi = channel(std::span<const std::string>(names),
                            "front.s000.xi");
    bad[xi] = std::numeric_limits<double>::infinity();
    rejects<std::runtime_error>([&] { st.set_tire_state(names, bad); },
                                "finite or NaN");
    // 'rejected set_state preserves the state wire' — the oracle's
    // assert_bitwise_equal leaf, asserted here as a raw-bytes comparison
    // since the dropped node carries no golden pair.
    const auto preserved = st.tire_state();
    bt::require(std::ranges::equal(std::as_bytes(std::span{preserved}),
                                   std::as_bytes(std::span{committed})),
                "rejected set_state preserves the state wire");
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                   "increasing timestamp");
    st.set_tire_state(names, committed);
    (void)st.tire_qfrc(dt);
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                   "increasing timestamp");
}

// native_distributed_tire/distributed_observations_declare_multi_contact
[[gnu::noinline]] void test_distributed_observations_declare_multi_contact() {
    const auto env = bike::research::make_environment(
            bundle("env_request") / "request.toml");
    runtime::RideControl const command;
    env->begin_control(command, 0., 0.);
    const auto result = env->advance_control(std::nullopt, std::nullopt);
    bt::require(result.has_value(), "first control window completes");
    env->acknowledge_control();
    const auto sample = env->snapshot().latest_sample;
    bt::require(sample != nullptr, "first interval publishes a sample");
    const auto find = [](const rt::WireObject &o,
                         std::string_view key) -> const rt::Wire * {
        for (const auto &[k, v] : o)
            if (k == key) return &v;
        return nullptr;
    };
    const auto *tires = find(sample->channels, "tires");
    bt::require(tires != nullptr, "channels carry tires");
    const auto &tires_obj = std::get<rt::WireObject>(tires->value);
    for (const char *side : {"front", "rear"}) {
        const auto *t = find(tires_obj, side);
        bt::require(t != nullptr, "tires channel carries the side");
        const auto &obj = std::get<rt::WireObject>(t->value);
        const auto *multi = find(obj, "supports_multiple_contacts");
        const auto *backend = find(obj, "backend");
        bt::require(multi != nullptr &&
                            std::get<bool>(multi->value),
                    "distributed side declares multi-contact");
        bt::require(backend != nullptr &&
                            std::get<std::string>(backend->value) ==
                                    "distributed_2d_reference",
                    "distributed backend name");
    }
}

// native_distributed_tire/distributed_trajectory_matches_python_channel_wise_bitwise
// DROPPED — the one-second env-pair channel-wise gate has no fixture
// form: the oracle compares the live Python reference env's per-step
// transition trees against the native env's. The native runtime has no
// recorded-trajectory replay seam and the Stepper-level parity half is
// covered bitwise by test_distributed_qfrc_bitwise's 40-step episode
// replay.

} // namespace

int main() {
    return bt::run([] {
        test_signed_distance_matches_python_bitwise();
        test_config_projection_carries_the_distributed_section();
        test_config_rejects_bad_distributed_sections();
        test_distributed_state_wire_round_trip();
        test_distributed_qfrc_bitwise();
        test_distributed_probe_and_clock();
        test_distributed_observations_declare_multi_contact();
    });
}
