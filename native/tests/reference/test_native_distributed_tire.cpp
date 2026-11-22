// test_native_distributed_tire.cpp — C++ port of
// tests/reference/test_native_distributed_tire.py (batch 3.13).
//
// The distributed_2d_reference backend: signed-profile distance bitwise,
// config projection/rejection, the station state wire, the golden
// episode replay (qfrc + evolved wire + snapshot/diagnostic float
// leaves), the once-per-timestamp clock and the committed-channel
// capability flags.
#include "tyre_reference_support.hpp"

#include "support/record_initial_state.hpp"

#include "io/json.hpp"
#include "pneumatic/profile.hpp"
#include "research/environment.hpp"
#include "runtime/research.hpp"
#include "terrain/profile.hpp"
#include "terrain/types.hpp"
#include "writers/distributed_tire.hpp"
#include "writers/tire_common.hpp"

#include <cmath>
#include <format>

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
    // Ban #39: the retained damping_ns_m key is pinned to zero —
    // rate-independent hysteresis carries the radial loss.
    mutate([](io::Value &v) {
        auto &dist = nested(v, "tire", "distributed");
        set_number(*member_mut(dist, "density"), "damping_ns_m", "40");
    });
    mutate([](io::Value &v) {
        auto &dist = nested(v, "tire", "distributed");
        set_number(*member_mut(dist, "density"),
                   "hysteresis_rate_eps_mps", "0");
    });
    mutate([](io::Value &v) {
        auto &dist = nested(v, "tire", "distributed");
        set_number(*member_mut(dist, "density"), "loss_factor", "1.0");
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
    bt::record_initial_state(bundle("env_request") / "request.toml");
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

// ---------------------------------------------------------------------
// Stage A surface physics tests (Spec §3.2, §4.2.2)
// ---------------------------------------------------------------------

struct StageADataDeleter {
    void operator()(mjData *data) const noexcept {
        if (data != nullptr) mj_deleteData(data);
    }
};

struct StageAStand {
    bike::model::ModelOwner model;
    std::unique_ptr<mjData, StageADataDeleter> data;

    explicit StageAStand(std::string_view xml)
        : model(bike::model::compile_mjcf(xml)),
          data(engine::make_data(model.get())) {
        engine::forward(model.get(), data.get());
    }
};

[[nodiscard]] std::unique_ptr<StageAStand> make_flat_wheel_stand(double dt) {
    const std::string xml =
            R"(<mujoco><compiler angle="radian"/><option timestep=")" +
            // {:.17g} roundtrips dt exactly; std::to_string rounds to 6 decimals (e.g. 1.25e-5 -> 1.3e-5).
            std::format("{:.17g}", dt) +
            R"(" gravity="0 0 0" integrator="Euler"/>
      <asset><hfield name="road" nrow="2" ncol="801" size="2 1 0.01 0.2"/></asset>
      <worldbody>
        <geom name="terrain" type="hfield" hfield="road" pos="0 0 0" contype="0" conaffinity="0"/>
        <body name="front_wheel" pos="0 0 0.37">
          <joint name="root_x" type="slide" axis="1 0 0"/>
          <joint name="root_z" type="slide" axis="0 0 1"/>
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
          <geom name="geom_front_contact" type="cylinder" size="0.37 0.025"
                euler="1.5707963267948966 0 0" contype="0" conaffinity="0" mass="0"/>
        </body>
        <body name="rear_wheel" pos="0 0 2.0">
          <geom name="geom_rear_contact" type="sphere" size="0.37" contype="0" conaffinity="0"/>
        </body>
      </worldbody>
    </mujoco>)";
    auto stand = std::make_unique<StageAStand>(xml);
    bt::require(stand->model->opt.timestep == dt,
                "wheel stand MuJoCo timestep equals the requested dt");
    return stand;
}

[[nodiscard]] nativecfg::SurfaceSpec
to_native_surface(const terrain::SurfaceSpec &s) {
    return {
        .name = s.name,
        .mu_peak = s.mu_peak,
        .mu_slide = s.mu_slide,
        .slip_stiffness_per_load = s.slip_stiffness_per_load,
        .stribeck_speed_mps = s.stribeck_speed_mps,
        .crr_delta = s.crr_delta,
        .load_sensitivity = s.load_sensitivity,
    };
}

struct StageATireOptions {
    int station_count;
    double tire_mu;
};

[[nodiscard]] nativecfg::TireConfig
stage_a_track_config(StageATireOptions options,
                     nativecfg::SurfaceMap surface_map) {
    const nativecfg::HingeDensity density{
        .knots_m = {0.0, 0.004, 0.012},
        .stiffness_n_m = {300000.0, 0.0, 0.0},
        .damping_ns_m = 0.0,
        .loss_factor = 0.05,
        .hysteresis_rate_eps_mps = 0.01,
        .provenance = "synthetic density; not fitted or measured",
    };
    const nativecfg::DistributedMaterial mat{
        .density = density,
        .fitting_dataset_id = "none-synthetic",
        .valid_load_range_n = {0.0, 2000.0},
        .valid_deflection_range_m = {0.0, 0.04},
        .calibration_status = "experimental_unvalidated",
    };
    nativecfg::TireConfig cfg;
    cfg.backend = "distributed_2d_reference";
    cfg.surface_mode = "track";
    cfg.front = {.material = std::nullopt,
                 .tangent_k_n_m = 20000.0,
                 .mu = options.tire_mu,
                 .relaxation_length_m = 0.2};
    cfg.rear = cfg.front;
    cfg.significant_delta_m = 0.0001;
    cfg.significance_fraction = 0.05;
    cfg.distinct_normal_deg = 20.0;
    cfg.surface_map = std::move(surface_map);
    cfg.distributed = nativecfg::DistributedConfig{
        .station_count = options.station_count,
        .density = density,
        .front = mat,
        .rear = mat,
    };
    return cfg;
}

struct StageAStandSolve {
    double dt_s;
    double target_load_n;
};

// Bisects static vertical compression depth at z-joint index 1 for target_load_n.
[[nodiscard]] double solve_stand_depth(const StageAStand &stand,
                                       DistributedTireWriter &tire,
                                       StageAStandSolve request) {
    const auto *m = stand.model.get();
    auto *d = stand.data.get();
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    std::ranges::fill(qpos, 0.0);
    std::ranges::fill(qvel, 0.0);
    double lo = 1e-7;
    double hi = 0.06;
    for (int iter = 0; iter < 60; ++iter) {
        const double mid = 0.5 * (lo + hi);
        qpos[1] = -mid;
        engine::forward(m, d);
        const double fz = tire.probe_qfrc(d, request.dt_s)[1];
        if (fz < request.target_load_n)
            lo = mid;
        else
            hi = mid;
    }
    qpos[1] = 0.0;
    engine::forward(m, d);
    return 0.5 * (lo + hi);
}

[[nodiscard]] double stand_kinetic_energy(const mjModel *m, const mjData *d) {
    std::vector<double> momentum(static_cast<std::size_t>(m->nv), 0.0);
    mj_mulM(m, d, momentum.data(), d->qvel);
    const auto qvel = model_access::readonly_buffer(d->qvel, m->nv, "qvel");
    double dot = 0.0;
    for (std::size_t i = 0; i < momentum.size(); ++i)
        dot += qvel[i] * momentum[i];
    return 0.5 * dot;
}

[[nodiscard]] double sum_patch_tangent(const TireSnapshot &snap) {
    double sum = 0.0;
    for (const TirePatch &p : snap.patches)
        sum += p.tangent_force_n;
    return sum;
}

// 1. Shear stiffness scaling (asphalt 15.0 vs scree 4.5 -> 0.3x on scree)
// at standstill (v_roll == 0) and rolling (v_roll > 0).
[[gnu::noinline]] void
test_stage_a_shear_stiffness_scaling_standstill_and_rolling() {
    constexpr double dt = 0.00125;
    constexpr int stations = 64;
    auto asphalt_stand = make_flat_wheel_stand(dt);
    auto scree_stand = make_flat_wheel_stand(dt);
    const nativecfg::SurfaceMap asphalt_map{
        .surface = to_native_surface(terrain::get_surface("asphalt")),
        .sections = {}};
    const nativecfg::SurfaceMap scree_map{
        .surface = to_native_surface(terrain::get_surface("scree")),
        .sections = {}};
    auto asphalt_tire = std::make_unique<DistributedTireWriter>(
            asphalt_stand->model.get(), asphalt_stand->data.get(),
            stage_a_track_config(
                    {.station_count = stations, .tire_mu = 2.0}, asphalt_map));
    auto scree_tire = std::make_unique<DistributedTireWriter>(
            scree_stand->model.get(), scree_stand->data.get(),
            stage_a_track_config(
                    {.station_count = stations, .tire_mu = 2.0}, scree_map));

    const double depth =
            solve_stand_depth(
                    *asphalt_stand, *asphalt_tire,
                    {.dt_s = dt, .target_load_n = 400.0});
    for (auto *ws : {asphalt_stand.get(), scree_stand.get()}) {
        const auto qpos = model_access::mutable_buffer(
                ws->data->qpos, ws->model->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(
                ws->data->qvel, ws->model->nv, "qvel");
        qpos[0] = 0.0;
        qpos[1] = -depth;
        qpos[2] = 0.0;
        qvel[0] = 0.0; // standstill: v_roll == 0
        qvel[1] = 0.0;
        qvel[2] = 0.005; // tiny spin well below friction limit
        engine::forward(ws->model.get(), ws->data.get());
    }

    const auto qfrc_asphalt_0 =
            asphalt_tire->qfrc(asphalt_stand->data.get(), dt);
    const auto qfrc_scree_0 =
            scree_tire->qfrc(scree_stand->data.get(), dt);
    bt::require(asphalt_tire->snapshots_committed() &&
                        scree_tire->snapshots_committed(),
                "standstill step commits snapshots");
    const double ft_asphalt_0 =
            sum_patch_tangent(asphalt_tire->snapshot_storage()[0]);
    const double ft_scree_0 =
            sum_patch_tangent(scree_tire->snapshot_storage()[0]);
    bt::require(std::abs(ft_asphalt_0) > 1e-3,
                "standstill micro-spin generates nonzero tangential force");
    bt::require(std::abs(ft_scree_0 - 0.3 * ft_asphalt_0) <= 1e-9,
                "standstill (v_roll == 0) scree tangential force equals 0.3x asphalt");
    bt::require(std::abs(qfrc_scree_0[0] - 0.3 * qfrc_asphalt_0[0]) <= 1e-9,
                "standstill horizontal qfrc[0] on scree equals 0.3x asphalt");

    // Rolling case (v_roll = 2.0 m/s > 0) with identical small slip step.
    asphalt_tire->reset();
    scree_tire->reset();
    const double r_eff = asphalt_tire->snapshot_storage()[0].effective_radius_m > 0.0
                                 ? asphalt_tire->snapshot_storage()[0].effective_radius_m
                                 : (0.37 - depth);
    for (auto *ws : {asphalt_stand.get(), scree_stand.get()}) {
        const auto qvel = model_access::mutable_buffer(
                ws->data->qvel, ws->model->nv, "qvel");
        qvel[0] = 2.0;
        qvel[1] = 0.0;
        qvel[2] = 2.0 / r_eff + 0.008; // small driving slip well below mu*Fn
        ws->data->time = 0.0;
        engine::forward(ws->model.get(), ws->data.get());
    }
    for (int step = 0; step < 4; ++step) {
        asphalt_stand->data->time = static_cast<double>(step + 1) * dt;
        scree_stand->data->time = static_cast<double>(step + 1) * dt;
        const auto qa = asphalt_tire->qfrc(asphalt_stand->data.get(), dt);
        const auto qs = scree_tire->qfrc(scree_stand->data.get(), dt);
        const double fta = sum_patch_tangent(asphalt_tire->snapshot_storage()[0]);
        const double fts = sum_patch_tangent(scree_tire->snapshot_storage()[0]);
        bt::require(std::abs(fta) > 1e-3,
                    "rolling micro-slip generates nonzero tangential force");
        bt::require(std::abs(fts - 0.3 * fta) <= 1e-9,
                    "rolling scree tangential force equals 0.3x asphalt");
        bt::require(std::abs(qs[0] - 0.3 * qa[0]) <= 1e-9,
                    "rolling horizontal qfrc[0] on scree equals 0.3x asphalt");
    }
}

// 2. Energy conservation across a blend_m = 0.4 transition (hardpack -> scree)
// and 4-column state()/set_state() roundtrip.
[[gnu::noinline]] void
test_stage_a_transition_energy_conservation_and_wire_roundtrip() {
    const nativecfg::SurfaceMap blend_map{
        .surface = to_native_surface(terrain::get_surface("hardpack")),
        .sections = {nativecfg::SurfaceSection{
            .start_m = 0.6,
            .end_m = 2.0,
            .surface = to_native_surface(terrain::get_surface("scree")),
            .blend_m = 0.4,
        }}};

    // Part A: single-station inside TransitionZone [0.4, 0.8] verifies
    // bitwise stored_energy(d) == elastic_energy_j() and 4-col wire roundtrip.
    {
        constexpr double dt = 0.00125;
        constexpr int stations = 16;
        auto ws = make_flat_wheel_stand(dt);
        const auto *m = ws->model.get();
        auto *d = ws->data.get();
        const auto cfg = stage_a_track_config(
                {.station_count = stations, .tire_mu = 2.0}, blend_map);
        auto tire = std::make_unique<DistributedTireWriter>(m, d, cfg);
        auto restored = std::make_unique<DistributedTireWriter>(m, d, cfg);

        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        qpos[0] = 0.55; // inside [0.4, 0.8] blend window (0.3 < r_k < 0.8)
        qpos[1] = -0.005;
        qpos[2] = 0.0;
        qvel[0] = 1.5;
        qvel[1] = 0.0;
        qvel[2] = 1.5 / 0.365 + 0.4;
        engine::forward(m, d);

        for (int step = 0; step < 3; ++step) {
            d->time = static_cast<double>(step + 1) * dt;
            (void)tire->qfrc(d, dt);
            bt::require(tire->brush_loss_step_j() >= 0.0,
                        "transition step brush loss is nonnegative");
            bt::require(tire->diagnostic_storage()[0].active_station_count == 1,
                        "16-station stand engages one bottom station");
            bt::require(tire->stored_energy(d) == tire->elastic_energy_j(),
                        "stored_energy(d) equals elastic_energy_j() bitwise in transition");
        }

        const auto wire = tire->state();
        bt::require(wire.size() == 8U * static_cast<std::size_t>(stations),
                    "4-column state wire size unchanged");
        restored->set_state(tire->state_names(), wire);
        const auto re_wire = restored->state();
        bt::require(std::ranges::equal(std::as_bytes(std::span{wire}),
                                       std::as_bytes(std::span{re_wire})),
                    "4-column state wire roundtrips bitwise");
        bt::require(restored->stored_energy(d) == tire->stored_energy(d),
                    "restored writer reproduces stored_energy(d) bitwise");
        d->time += dt;
        const auto q_orig = tire->qfrc(d, dt);
        const auto q_rest = restored->qfrc(d, dt);
        bt::require(q_orig == q_rest,
                    "restored writer reproduces next-step qfrc bitwise");
    }

    // Part B: 64-station loaded wheel dynamically rolling with shear deflection
    // across hardpack -> scree [0.4, 0.8] (blend_m = 0.4) satisfies discrete
    // passivity and strict energy balance (< 1e-3 J).
    {
        constexpr double dt = 0.0000125;
        constexpr int stations = 64;
        constexpr double load = 480.0;
        auto ws = make_flat_wheel_stand(dt);
        const auto *m = ws->model.get();
        auto *d = ws->data.get();
        auto tire = std::make_unique<DistributedTireWriter>(
                m, d, stage_a_track_config(
                              {.station_count = stations, .tire_mu = 2.0},
                              blend_map));

        const double depth = solve_stand_depth(
                *ws, *tire, {.dt_s = dt, .target_load_n = load});
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        const auto qfrc_applied = model_access::mutable_buffer(
                d->qfrc_applied, m->nv, "qfrc_applied");
        qpos[0] = 0.25; // starts on pure hardpack (< 0.4 m)
        qpos[1] = -depth;
        qpos[2] = 0.0;
        qvel[0] = 2.5;
        qvel[1] = 0.0;
        qvel[2] = 2.5 / (0.37 - depth) + 2.5; // brisk spin slip builds shear energy
        engine::forward(m, d);

        const double e_initial =
                stand_kinetic_energy(m, d) + tire->stored_energy(d) + load * qpos[1];
        double total_dissipated = 0.0;
        double max_energy_error = 0.0;
        for (int step = 0; step < 24000; ++step) {
            const auto force = tire->qfrc(d, dt);
            bt::require(tire->brush_loss_step_j() >= 0.0,
                        "brush_loss_step_j >= 0 across blend transition");
            bt::require(tire->radial_dissipation_power_w() >= 0.0,
                        "radial_dissipation_power_w >= 0 across blend transition");
            bt::require(std::abs(tire->stored_energy(d) -
                                 tire->elastic_energy_j()) <= 1e-14,
                        "multi-station stored_energy(d) matches elastic_energy_j()");
            total_dissipated += tire->brush_loss_step_j() +
                                tire->radial_dissipation_power_w() * dt;
            std::ranges::copy(force, qfrc_applied.begin());
            qfrc_applied[1] -= load;
            engine::step(m, d);
            engine::forward(m, d);
            const double e_now = stand_kinetic_energy(m, d) +
                                 tire->stored_energy(d) + load * qpos[1];
            const double residual = (e_now - e_initial) + total_dissipated;
            max_energy_error = std::max(max_energy_error, std::abs(residual));
        }
        bt::require(std::abs(d->time - 24000 * dt) <= 1e-9,
                    "wheel stand model time matches the requested 0.3 s duration");
        bt::require(qpos[0] > 0.85,
                    "wheel rolled completely across [0.4, 0.8] transition onto scree");
        bt::require(tire->diagnostic_storage()[0].surface == "scree",
                    "wheel ends on scree surface");
        bt::require(max_energy_error < 1e-3,
                    "energy ledger across blend_m = 0.4 transition balances within 1e-3 J");
    }
}

// 3a. Wheel-level load sensitivity on scree: exact Stribeck at < 500 N,
// saturation below 0.45 at > 800 N (even though per-station load < 500 N),
// and strict growth of mu_eff(F_z) * F_z.
[[gnu::noinline]] void
test_stage_a_wheel_load_sensitivity_and_monotonic_traction() {
    constexpr double dt = 0.00125;
    constexpr int stations = 64;
    const nativecfg::SurfaceMap scree_map{
        .surface = to_native_surface(terrain::get_surface("scree")),
        .sections = {}};

    // Sub-threshold wheel load (< 500 N): matches Stribeck exactly.
    {
        auto ws = make_flat_wheel_stand(dt);
        const auto *m = ws->model.get();
        auto *d = ws->data.get();
        auto tire = std::make_unique<DistributedTireWriter>(
                m, d, stage_a_track_config(
                              {.station_count = stations, .tire_mu = 2.0},
                              scree_map));
        const double depth = solve_stand_depth(
                *ws, *tire, {.dt_s = dt, .target_load_n = 400.0});
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        qpos[1] = -depth;
        qvel[0] = 1.2; // pure horizontal slide (omega = 0) -> uniform slip = 1.2 m/s
        qvel[2] = 0.0;
        engine::forward(m, d);
        (void)tire->qfrc(d, dt);
        const auto &diag = tire->diagnostic_storage()[0];
        bt::require(diag.normal_load_n > 350.0 && diag.normal_load_n < 500.0,
                    "sub-threshold wheel load lies in (350, 500) N");
        const double want_stribeck =
                0.38 + (0.45 - 0.38) * std::exp(-1.2 / 4.5);
        bt::require(std::abs(diag.friction_coefficient - want_stribeck) <= 1e-15,
                    "below 500 N wheel load, friction_coefficient matches Stribeck");
        bt::require(diag.surface == "scree", "surface diagnostic names scree");
    }

    // High wheel load (> 800 N): saturates below 0.45 despite every individual
    // station carrying < 500 N.
    {
        auto ws = make_flat_wheel_stand(dt);
        const auto *m = ws->model.get();
        auto *d = ws->data.get();
        auto tire = std::make_unique<DistributedTireWriter>(
                m, d, stage_a_track_config(
                              {.station_count = stations, .tire_mu = 2.0},
                              scree_map));
        const double depth = solve_stand_depth(
                *ws, *tire, {.dt_s = dt, .target_load_n = 900.0});
        const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        qpos[1] = -depth;
        qvel[0] = 0.0;
        qvel[2] = 0.0;
        engine::forward(m, d);
        (void)tire->qfrc(d, dt);
        const auto &diag = tire->diagnostic_storage()[0];
        const auto &snap = tire->snapshot_storage()[0];
        bt::require(diag.normal_load_n > 800.0,
                    "high-load stand exceeds 800 N total wheel load");
        for (const TirePatch &p : snap.patches)
            bt::require(p.normal_load_n < 500.0,
                        "each individual station carries < 500 N (proves wheel-level F_z)");
        bt::require(diag.friction_coefficient > 0.0 &&
                            diag.friction_coefficient < 0.38 + (0.45 - 0.38),
                    "above 800 N on scree, friction_coefficient saturates below 0.45");
        const double expected_mu =
                0.45 * tirecommon::load_sensitivity_factor(diag.normal_load_n, 0.15);
        bt::require(std::abs(diag.friction_coefficient - expected_mu) <= 1e-14,
                    "high-load friction_coefficient matches phi_load(F_z_wheel) * mu_peak");
    }

    // Strict monotonicity of mu_eff(F_z) * F_z across wheel loads.
    double prev_cap = 0.0;
    for (const double target_load : {300.0, 500.0, 700.0, 950.0, 1300.0}) {
        auto ws = make_flat_wheel_stand(dt);
        const auto *m = ws->model.get();
        auto *d = ws->data.get();
        auto tire = std::make_unique<DistributedTireWriter>(
                m, d, stage_a_track_config(
                              {.station_count = stations, .tire_mu = 2.0},
                              scree_map));
        const double depth = solve_stand_depth(
                *ws, *tire, {.dt_s = dt, .target_load_n = target_load});
        model_access::mutable_buffer(d->qpos, m->nq, "qpos")[1] = -depth;
        engine::forward(m, d);
        (void)tire->qfrc(d, dt);
        const auto &diag = tire->diagnostic_storage()[0];
        const double cap = diag.friction_coefficient * diag.normal_load_n;
        bt::require(cap > prev_cap,
                    "wheel traction limit mu_eff(F_z) * F_z strictly increases");
        prev_cap = cap;
    }
}

// 3b. World rolling moment (crr_delta), absence of artificial internal caliper
// reaction on frame/root pitch, rollback passivity (v_x < 0, omega > 0), and
// >16-char surface capacity preservation across reset() + try_compute_into().
[[gnu::noinline]] void
test_stage_a_world_rolling_moment_no_caliper_reaction_and_reset_capacity() {
    constexpr double dt = 0.00125;
    constexpr int stations = 64;
    // 4-DOF stand: frame body with (root_x, root_z, root_pitch) and concentric
    // child front_wheel body with front_wheel_spin hinge. Any internal caliper
    // torque between frame and wheel would accelerate root_pitch, whereas a
    // pure world rolling couple on front_wheel leaves root_pitch acceleration
    // completely unchanged.
    const std::string xml = R"(<mujoco>
      <compiler angle="radian"/>
      <option timestep="0.00125" gravity="0 0 0" integrator="Euler"/>
      <asset>
        <hfield name="road" nrow="2" ncol="201" size="1 1 0.01 0.2"/>
      </asset>
      <worldbody>
        <geom name="terrain" type="hfield" hfield="road" pos="0 0 0" contype="0" conaffinity="0"/>
        <body name="frame" pos="0 0 0.37">
          <joint name="root_x" type="slide" axis="1 0 0"/>
          <joint name="root_z" type="slide" axis="0 0 1"/>
          <joint name="root_pitch" type="hinge" axis="0 1 0"/>
          <inertial mass="10.0" pos="0 0 0" diaginertia="1.0 1.2 1.0"/>
          <body name="front_wheel" pos="0 0 0">
            <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
            <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
            <geom name="geom_front_contact" type="cylinder" size="0.37 0.025"
                  euler="1.5707963267948966 0 0" contype="0" conaffinity="0" mass="0"/>
          </body>
        </body>
        <body name="rear_wheel" pos="0 0 2.0">
          <geom name="geom_rear_contact" type="sphere" size="0.37" contype="0" conaffinity="0"/>
        </body>
      </worldbody>
    </mujoco>)";

    auto stand_scree = std::make_unique<StageAStand>(xml);
    auto stand_nocrr = std::make_unique<StageAStand>(xml);

    const nativecfg::SurfaceSpec scree_spec =
            to_native_surface(terrain::get_surface("scree"));
    nativecfg::SurfaceSpec nocrr_spec = scree_spec;
    nocrr_spec.crr_delta = 0.0;

    auto tire_scree = std::make_unique<DistributedTireWriter>(
            stand_scree->model.get(), stand_scree->data.get(),
            stage_a_track_config({.station_count = stations, .tire_mu = 2.0},
                                 {.surface = scree_spec, .sections = {}}));
    auto tire_nocrr = std::make_unique<DistributedTireWriter>(
            stand_nocrr->model.get(), stand_nocrr->data.get(),
            stage_a_track_config({.station_count = stations, .tire_mu = 2.0},
                                 {.surface = nocrr_spec, .sections = {}}));

    const double depth =
            solve_stand_depth(
                    *stand_scree, *tire_scree,
                    {.dt_s = dt, .target_load_n = 600.0});
    for (auto *s : {stand_scree.get(), stand_nocrr.get()}) {
        const auto qpos = model_access::mutable_buffer(
                s->data->qpos, s->model->nq, "qpos");
        const auto qvel = model_access::mutable_buffer(
                s->data->qvel, s->model->nv, "qvel");
        qpos[0] = 0.0;
        qpos[1] = -depth;
        qpos[2] = 0.05; // nonzero frame pitch angle
        qpos[3] = 0.0;
        qvel[0] = 3.0;
        qvel[1] = 0.0;
        qvel[2] = 0.2; // nonzero frame pitch rate
        qvel[3] = 8.0; // forward wheel spin (omega_world = 8.2 rad/s > 0)
        engine::forward(s->model.get(), s->data.get());
    }

    const auto qfrc_scree = tire_scree->qfrc(stand_scree->data.get(), dt);
    const auto qfrc_nocrr = tire_nocrr->qfrc(stand_nocrr->data.get(), dt);
    const auto &snap = tire_scree->snapshot_storage()[0];
    const auto &diag = tire_scree->diagnostic_storage()[0];

    double couple_y_sum = 0.0;
    for (const TirePatch &p : snap.patches) {
        bt::require(p.couple_world_nm[0] == 0.0 && p.couple_world_nm[2] == 0.0,
                    "planar rolling couple acts strictly about world Y");
        bt::require(p.couple_world_nm[1] < 0.0,
                    "forward-spinning wheel on scree receives negative couple_world_nm[1]");
        couple_y_sum += p.couple_world_nm[1];
    }
    const double expected_m_roll =
            -scree_spec.crr_delta * diag.normal_load_n * snap.effective_radius_m *
            std::tanh(8.2 / tirecommon::kWheelRollingTaperRadps);
    bt::require(std::abs(couple_y_sum - expected_m_roll) <= 1e-13,
                "sum of patch couple_world_nm[1] matches exact world rolling moment");

    // Exact generalized force reconstruction: qfrc == sum(J_p^T F_patch + J_r^T couple_patch).
    const auto *m = stand_scree->model.get();
    auto *d = stand_scree->data.get();
    const int wheel_body =
            model_access::resolve_id(m, mjOBJ_BODY, "front_wheel", true);
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    std::vector<double> reconstructed(nv, 0.0);
    std::vector<mjtNum> jp(3 * nv, 0.0), jr(3 * nv, 0.0);
    for (const TirePatch &p : snap.patches) {
        mj_jac(m, d, jp.data(), jr.data(), p.point_m.data(), wheel_body);
        const std::array<double, 3> tangent = {p.normal[2], 0.0, -p.normal[0]};
        const std::array<double, 3> f_world = {
            p.normal_load_n * p.normal[0] + p.tangent_force_n * tangent[0],
            p.normal_load_n * p.normal[1] + p.tangent_force_n * tangent[1],
            p.normal_load_n * p.normal[2] + p.tangent_force_n * tangent[2],
        };
        for (std::size_t i = 0; i < nv; ++i) {
            for (std::size_t k = 0; k < 3; ++k) {
                reconstructed[i] += jp[k * nv + i] * f_world[k] +
                                    jr[k * nv + i] * p.couple_world_nm[k];
            }
        }
    }
    for (std::size_t i = 0; i < nv; ++i)
        bt::require(std::abs(qfrc_scree[i] - reconstructed[i]) <= 1e-12,
                    "qfrc equals sum(J_p^T F_patch + J_r^T couple_patch) on every DOF");

    // Dynamic proof of zero internal caliper reaction on frame pitch:
    std::ranges::copy(qfrc_scree,
                      model_access::mutable_buffer(
                              stand_scree->data->qfrc_applied, m->nv, "qfrc_applied")
                              .begin());
    std::ranges::copy(qfrc_nocrr,
                      model_access::mutable_buffer(
                              stand_nocrr->data->qfrc_applied, m->nv, "qfrc_applied")
                              .begin());
    engine::forward(stand_scree->model.get(), stand_scree->data.get());
    engine::forward(stand_nocrr->model.get(), stand_nocrr->data.get());
    const auto qacc_scree = model_access::readonly_buffer(
            stand_scree->data->qacc, m->nv, "qacc");
    const auto qacc_nocrr = model_access::readonly_buffer(
            stand_nocrr->data->qacc, m->nv, "qacc");
    bt::require(std::abs(qacc_scree[2] - qacc_nocrr[2]) <= 1e-13,
                "crr_delta world couple induces zero artificial reaction on frame root_pitch");
    bt::require(qacc_scree[3] < qacc_nocrr[3] - 1.0,
                "crr_delta world couple decelerates front_wheel_spin");

    // Rollback with forward wheel spin (v_x < 0, omega > 0): strictly passive.
    {
        const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
        qvel[0] = -0.6; // rollback v_x < 0
        qvel[1] = 0.0;
        qvel[2] = 0.0;
        qvel[3] = 4.5;  // forward wheel spin omega > 0
        d->time += dt;
        engine::forward(m, d);
        (void)tire_scree->qfrc(d, dt);
        bt::require(tire_scree->brush_loss_step_j() > 0.0,
                    "rollback with forward spin (v_x < 0, omega > 0) is strictly passive");
    }

    // Long (>16-char, non-SSO) surface name capacity preservation across reset()
    // and try_compute_into() on both loaded front and airborne rear wheels.
    {
        nativecfg::SurfaceSpec long_spec = scree_spec;
        long_spec.name = "scree_extended_custom_surface_32";
        auto tire_long = std::make_unique<DistributedTireWriter>(
                m, d,
                stage_a_track_config(
                                     {.station_count = stations, .tire_mu = 2.0},
                                     {.surface = long_spec, .sections = {}}));
        const char *front_ptr = tire_long->diagnostic_storage()[0].surface.data();
        const char *rear_ptr = tire_long->diagnostic_storage()[1].surface.data();
        const std::size_t front_cap =
                tire_long->diagnostic_storage()[0].surface.capacity();
        bt::require(front_cap >= long_spec.name.size(),
                    "constructor reserves full >16-char surface name capacity");

        std::vector<double> out(nv, 0.0);
        d->time += dt;
        bt::require(tire_long->try_compute_into(d, dt, out) == CoreStatus::ok,
                    "try_compute_into succeeds with >16-char surface name");
        bt::require(tire_long->diagnostic_storage()[0].surface == long_spec.name &&
                            tire_long->diagnostic_storage()[1].surface == long_spec.name,
                    "both loaded front and airborne rear diagnostics report surface name");
        bt::require(tire_long->diagnostic_storage()[0].surface.data() == front_ptr &&
                            tire_long->diagnostic_storage()[1].surface.data() == rear_ptr,
                    "first try_compute_into reuses pre-reserved surface buffers");

        tire_long->reset();
        bt::require(tire_long->diagnostic_storage()[0].surface.capacity() == front_cap &&
                            tire_long->diagnostic_storage()[0].surface.data() == front_ptr &&
                            tire_long->diagnostic_storage()[1].surface.data() == rear_ptr,
                    "reset() preserves >16-char surface buffer capacity without shrinking");

        d->time += dt;
        bt::require(tire_long->try_compute_into(d, dt, out) == CoreStatus::ok,
                    "post-reset try_compute_into succeeds");
        bt::require(tire_long->diagnostic_storage()[0].surface == long_spec.name &&
                            tire_long->diagnostic_storage()[0].surface.data() == front_ptr &&
                            tire_long->diagnostic_storage()[1].surface.data() == rear_ptr,
                    "post-reset try_compute_into reuses exact same heap buffers (0 alloc)");
    }
}

// 5. Boundary contract: a finite nonnegative crr_delta accepted by validate
// (e.g. DBL_MAX) that overflows the rolling-couple math must be rejected by
// try_compute_into as invalid_input (std::overflow_error mapping), NOT ok,
// and must leave committed state/energy/loss/snapshot and the clock untouched.
[[gnu::noinline]] void
test_stage_a_rolling_crr_overflow_rejected_and_state_atomic() {
    constexpr double dt = 0.00125;
    constexpr int stations = 64;
    auto ws = make_flat_wheel_stand(dt);
    const auto *m = ws->model.get();
    auto *d = ws->data.get();
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    const std::size_t nv = static_cast<std::size_t>(m->nv);

    const nativecfg::SurfaceSpec scree_spec =
            to_native_surface(terrain::get_surface("scree"));
    // Normal-control writer (registry-scale crr) solves the 480 N depth; never
    // probe the extreme config (its couple overflows even at omega == 0).
    auto control_tire = std::make_unique<DistributedTireWriter>(
            m, d, stage_a_track_config(
                          {.station_count = stations, .tire_mu = 2.0},
                                       {.surface = scree_spec, .sections = {}}));
    const double depth = solve_stand_depth(
            *ws, *control_tire, {.dt_s = dt, .target_load_n = 480.0});
    bt::require(depth > 1e-6, "480 N stand depth is positive (load achievable)");

    // Extreme config: crr_delta = DBL_MAX is finite and nonnegative, so the
    // existing validate accepts it (construction must succeed, else vacuous).
    nativecfg::SurfaceSpec extreme_spec = scree_spec;
    extreme_spec.crr_delta = std::numeric_limits<double>::max();
    auto extreme_tire = std::make_unique<DistributedTireWriter>(
            m, d, stage_a_track_config(
                          {.station_count = stations, .tire_mu = 2.0},
                                       {.surface = extreme_spec, .sections = {}}));

    // Valid registry-scale control at the loaded pose: finite ok + finite
    // out/couple/loss, and real wheel load present so the rolling channel runs.
    qpos[0] = 0.0;
    qpos[1] = -depth;
    qpos[2] = 0.0;
    qvel[0] = 2.5;
    qvel[1] = 0.0;
    qvel[2] = 2.5 / (0.37 - depth) + 0.5;
    d->time = dt;
    engine::forward(m, d);
    std::vector<double> ctrl_out(nv, 0.0);
    bt::require(control_tire->try_compute_into(d, dt, ctrl_out) == CoreStatus::ok,
                "registry-scale crr control: loaded step returns ok");
    for (const double v : ctrl_out)
        bt::require(std::isfinite(v), "registry-scale crr control: finite qfrc");
    bt::require(control_tire->diagnostic_storage()[0].normal_load_n > 350.0,
                "loaded pose carries real wheel load (>350 N)");
    bt::require(std::isfinite(control_tire->brush_loss_step_j()),
                "registry-scale crr control: finite brush loss");
    for (const TirePatch &p : control_tire->snapshot_storage()[0].patches)
        for (const double c : p.couple_world_nm)
            bt::require(std::isfinite(c),
                        "registry-scale crr control: finite patch couple");

    // (1) Valid finite UNLOADED step for the extreme-config writer: wheel
    // airborne -> no stations -> rolling channel skipped -> finite ok. This
    // also proves the extreme config was accepted by validate.
    qpos[1] = 0.5;
    qvel[0] = 0.0;
    qvel[2] = 0.0;
    d->time = dt;
    engine::forward(m, d);
    std::vector<double> out(nv, 0.0);
    bt::require(extreme_tire->try_compute_into(d, dt, out) == CoreStatus::ok,
                "extreme crr config accepted: unloaded airborne step is finite ok");
    const auto state_before = extreme_tire->state();
    const double energy_before = extreme_tire->elastic_energy_j();
    const double loss_before = extreme_tire->brush_loss_step_j();
    const double radial_before = extreme_tire->radial_dissipation_power_w();
    const auto snap_before = extreme_tire->snapshot_storage()[0];
    const double snap_time_before = snap_before.time_s;
    const bool snap_contact_before = snap_before.geometric_contact;
    const std::size_t snap_patches_before = snap_before.patches.size();
    const double snap_radius_before = snap_before.effective_radius_m;
    const double diag_load_before =
            extreme_tire->diagnostic_storage()[0].normal_load_n;

    // (2) Move wheel to the loaded state, advance time, attempt the overflowing
    // update: load * DBL_MAX -> inf couple/loss/out -> must reject invalid_input.
    qpos[1] = -depth;
    qvel[0] = 2.5;
    qvel[2] = 2.5 / (0.37 - depth) + 0.5;
    d->time = 2 * dt;
    engine::forward(m, d);
    bt::require(extreme_tire->try_compute_into(d, dt, out) ==
                        CoreStatus::invalid_input,
                "extreme crr overflow rejected as invalid_input, not ok");

    // (3) Published state/energy/loss/snapshot remain unchanged by the reject.
    const auto state_after = extreme_tire->state();
    bt::require(std::ranges::equal(std::as_bytes(std::span{state_after}),
                                   std::as_bytes(std::span{state_before})),
                "rejected overflow leaves the state wire unchanged");
    bt::require(extreme_tire->elastic_energy_j() == energy_before,
                "rejected overflow leaves elastic energy unchanged");
    bt::require(extreme_tire->brush_loss_step_j() == loss_before,
                "rejected overflow leaves brush loss unchanged");
    bt::require(extreme_tire->radial_dissipation_power_w() == radial_before,
                "rejected overflow leaves radial power unchanged");
    const auto &snap_after = extreme_tire->snapshot_storage()[0];
    bt::require(snap_after.time_s == snap_time_before &&
                        snap_after.geometric_contact == snap_contact_before &&
                        snap_after.patches.size() == snap_patches_before &&
                        snap_after.effective_radius_m == snap_radius_before,
                "rejected overflow leaves the published snapshot unchanged");
    bt::require(extreme_tire->diagnostic_storage()[0].normal_load_n ==
                        diag_load_before,
                "rejected overflow leaves published diagnostics unchanged");

    // (4) Put the wheel airborne again and retry the SAME rejected timestamp:
    // success proves the rejected step did not advance the interval clock.
    qpos[1] = 0.5;
    qvel[0] = 0.0;
    qvel[2] = 0.0;
    engine::forward(m, d);
    bt::require(extreme_tire->try_compute_into(d, dt, out) == CoreStatus::ok,
                "retry at the SAME rejected timestamp succeeds airborne (clock not advanced)");
}

// 6. Cumulative boundary: every per-wheel rolling scalar can stay finite
// while the cross-wheel loss sum overflows (crr_delta=1e305 at ~480 N per
// wheel, omega=8 rad/s, dt=1 s -> ~1.4e308 J per wheel, two wheels -> inf).
// try_compute_into must reject the assembled result as invalid_input and
// leave committed state/energy/loss/snapshots/diagnostics and the clock
// untouched.
[[gnu::noinline]] void
test_stage_a_two_wheel_cumulative_loss_overflow_rejected_and_state_atomic() {
    constexpr double dt = 1.0;
    constexpr int stations = 64;
    constexpr double omega = 8.0;
    // The flat stand's rear wheel is a static airborne sphere; this fixture
    // gives BOTH wheels x/z slides and a spin hinge. Identical cylinders and
    // front==rear config make the load-depth curve identical per wheel.
    const std::string xml =
            R"(<mujoco><compiler angle="radian"/><option timestep=")" +
            std::format("{:.17g}", dt) +
            R"(" gravity="0 0 0" integrator="Euler"/>
      <asset><hfield name="road" nrow="2" ncol="801" size="2 1 0.01 0.2"/></asset>
      <worldbody>
        <geom name="terrain" type="hfield" hfield="road" pos="0 0 0" contype="0" conaffinity="0"/>
        <body name="front_wheel" pos="0 0 0.37">
          <joint name="front_x" type="slide" axis="1 0 0"/>
          <joint name="front_z" type="slide" axis="0 0 1"/>
          <joint name="front_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
          <geom name="geom_front_contact" type="cylinder" size="0.37 0.025"
                euler="1.5707963267948966 0 0" contype="0" conaffinity="0" mass="0"/>
        </body>
        <body name="rear_wheel" pos="1 0 0.37">
          <joint name="rear_x" type="slide" axis="1 0 0"/>
          <joint name="rear_z" type="slide" axis="0 0 1"/>
          <joint name="rear_wheel_spin" type="hinge" axis="0 1 0"/>
          <inertial mass="2.4" pos="0 0 0" diaginertia="0.15 0.30 0.15"/>
          <geom name="geom_rear_contact" type="cylinder" size="0.37 0.025"
                euler="1.5707963267948966 0 0" contype="0" conaffinity="0" mass="0"/>
        </body>
      </worldbody>
    </mujoco>)";
    auto ws = std::make_unique<StageAStand>(xml);
    bt::require(ws->model->opt.timestep == dt,
                "two-wheel stand MuJoCo timestep equals the requested dt");
    const auto *m = ws->model.get();
    auto *d = ws->data.get();
    const auto qpos = model_access::mutable_buffer(d->qpos, m->nq, "qpos");
    const auto qvel = model_access::mutable_buffer(d->qvel, m->nv, "qvel");
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    std::ranges::fill(qpos, 0.0);
    std::ranges::fill(qvel, 0.0);

    const nativecfg::SurfaceSpec scree_spec =
            to_native_surface(terrain::get_surface("scree"));
    auto control_tire = std::make_unique<DistributedTireWriter>(
            m, d, stage_a_track_config(
                          {.station_count = stations, .tire_mu = 2.0},
                                       {.surface = scree_spec, .sections = {}}));
    const double depth = solve_stand_depth(
            *ws, *control_tire, {.dt_s = dt, .target_load_n = 480.0});
    bt::require(depth > 1e-6, "480 N stand depth is positive (load achievable)");

    // crr_delta=1e305 is finite and nonnegative, so validate accepts it; one
    // wheel's rolling loss stays below DBL_MAX while two overflow the sum.
    constexpr double extreme_crr = 1e305;
    nativecfg::SurfaceSpec extreme_spec = scree_spec;
    extreme_spec.crr_delta = extreme_crr;
    auto extreme_tire = std::make_unique<DistributedTireWriter>(
            m, d, stage_a_track_config(
                          {.station_count = stations, .tire_mu = 2.0},
                                       {.surface = extreme_spec, .sections = {}}));

    const auto set_front = [&](bool loaded) {
        qpos[1] = loaded ? -depth : 0.5;
        qvel[2] = loaded ? omega : 0.0;
        engine::forward(m, d);
    };
    const auto set_rear = [&](bool loaded) {
        qpos[4] = loaded ? -depth : 0.5;
        qvel[5] = loaded ? omega : 0.0;
        engine::forward(m, d);
    };

    // Registry-scale control: the two-wheel loaded pose is valid and BOTH
    // wheels carry real load through the rolling channel.
    d->time = dt;
    set_front(true);
    set_rear(true);
    std::vector<double> ctrl_out(nv, 0.0);
    bt::require(control_tire->try_compute_into(d, dt, ctrl_out) == CoreStatus::ok,
                "registry-scale crr control: two-wheel loaded step returns ok");
    for (const double v : ctrl_out)
        bt::require(std::isfinite(v), "registry-scale crr control: finite qfrc");
    for (std::size_t s = 0; s < 2; ++s) {
        bt::require(control_tire->diagnostic_storage()[s].normal_load_n > 350.0,
                    "both wheels carry real load (>350 N) at the shared depth");
        bt::require(std::isfinite(control_tire->diagnostic_storage()[s].brush_loss_j),
                    "registry-scale crr control: finite per-wheel loss");
    }
    bt::require(std::isfinite(control_tire->brush_loss_step_j()),
                "registry-scale crr control: finite total loss");

    // Per-wheel rolling loss predicted from the control pose (loads and
    // effective radius do not depend on crr_delta): finite, and above
    // DBL_MAX/2 so two such losses overflow the cross-wheel sum.
    const double per_wheel_loss =
            extreme_crr * control_tire->diagnostic_storage()[0].normal_load_n *
            control_tire->snapshot_storage()[0].effective_radius_m *
            std::tanh(omega / tirecommon::kWheelRollingTaperRadps) * omega * dt;
    bt::require(std::isfinite(per_wheel_loss) && per_wheel_loss > 0.0,
                "one wheel's rolling loss is finite and positive");
    bt::require(per_wheel_loss > 0.5 * std::numeric_limits<double>::max(),
                "two per-wheel losses overflow the cross-wheel sum");

    std::vector<double> out(nv, 0.0);
    const auto single_wheel_ok = [&](std::size_t loaded_side) {
        bt::require(extreme_tire->try_compute_into(d, dt, out) == CoreStatus::ok,
                    "single-wheel loaded extreme step is finite ok");
        bt::require(extreme_tire->diagnostic_storage()[loaded_side].normal_load_n >
                            350.0,
                    "single-wheel extreme step loads the intended wheel");
        bt::require(extreme_tire->diagnostic_storage()[1 - loaded_side]
                            .normal_load_n == 0.0,
                    "single-wheel extreme step leaves the other wheel airborne");
        const double side_loss =
                extreme_tire->diagnostic_storage()[loaded_side].brush_loss_j;
        bt::require(std::isfinite(side_loss) &&
                            std::abs(side_loss - per_wheel_loss) <=
                                    1e-12 * per_wheel_loss,
                    "single-wheel per-wheel loss matches the finite prediction");
        bt::require(std::isfinite(extreme_tire->brush_loss_step_j()),
                    "single-wheel extreme total loss is finite");
        for (const TirePatch &p :
             extreme_tire->snapshot_storage()[loaded_side].patches)
            for (const double c : p.couple_world_nm)
                bt::require(std::isfinite(c),
                            "single-wheel extreme patch couple is finite");
    };

    // (1) Each wheel ALONE at the extreme crr passes every per-wheel guard
    // and commits the finite ~1.4e308 J loss — the later pair rejection is
    // the cumulative sum, not a per-wheel guard.
    d->time = dt;
    set_front(true);
    set_rear(false);
    single_wheel_ok(0);
    d->time = 2 * dt;
    set_front(false);
    set_rear(true);
    single_wheel_ok(1);

    const auto state_before = extreme_tire->state();
    const double energy_before = extreme_tire->elastic_energy_j();
    const double loss_before = extreme_tire->brush_loss_step_j();
    const double radial_before = extreme_tire->radial_dissipation_power_w();
    const auto snap0_before = extreme_tire->snapshot_storage()[0];
    const auto snap1_before = extreme_tire->snapshot_storage()[1];
    const double diag0_load_before =
            extreme_tire->diagnostic_storage()[0].normal_load_n;
    const double diag1_load_before =
            extreme_tire->diagnostic_storage()[1].normal_load_n;
    const double diag0_loss_before =
            extreme_tire->diagnostic_storage()[0].brush_loss_j;
    const double diag1_loss_before =
            extreme_tire->diagnostic_storage()[1].brush_loss_j;

    // (2) Both wheels loaded: each rolling value is finite (proved above)
    // but total_loss = front + rear -> inf. Must reject, not commit inf.
    d->time = 3 * dt;
    set_front(true);
    set_rear(true);
    bt::require(extreme_tire->try_compute_into(d, dt, out) ==
                        CoreStatus::invalid_input,
                "two-wheel cumulative loss overflow rejected as invalid_input, not ok");

    // (3) Published state/energy/loss/snapshots/diagnostics unchanged.
    const auto state_after = extreme_tire->state();
    bt::require(std::ranges::equal(std::as_bytes(std::span{state_after}),
                                   std::as_bytes(std::span{state_before})),
                "rejected cumulative overflow leaves the state wire unchanged");
    bt::require(extreme_tire->elastic_energy_j() == energy_before,
                "rejected cumulative overflow leaves elastic energy unchanged");
    bt::require(extreme_tire->brush_loss_step_j() == loss_before,
                "rejected cumulative overflow leaves brush loss unchanged");
    bt::require(extreme_tire->radial_dissipation_power_w() == radial_before,
                "rejected cumulative overflow leaves radial power unchanged");
    const auto &snap0_after = extreme_tire->snapshot_storage()[0];
    const auto &snap1_after = extreme_tire->snapshot_storage()[1];
    bt::require(snap0_after.time_s == snap0_before.time_s &&
                        snap0_after.geometric_contact == snap0_before.geometric_contact &&
                        snap0_after.patches.size() == snap0_before.patches.size() &&
                        snap0_after.effective_radius_m == snap0_before.effective_radius_m,
                "rejected cumulative overflow leaves the front snapshot unchanged");
    bt::require(snap1_after.time_s == snap1_before.time_s &&
                        snap1_after.geometric_contact == snap1_before.geometric_contact &&
                        snap1_after.patches.size() == snap1_before.patches.size() &&
                        snap1_after.effective_radius_m == snap1_before.effective_radius_m,
                "rejected cumulative overflow leaves the rear snapshot unchanged");
    bt::require(extreme_tire->diagnostic_storage()[0].normal_load_n ==
                        diag0_load_before &&
                        extreme_tire->diagnostic_storage()[1].normal_load_n ==
                                diag1_load_before,
                "rejected cumulative overflow leaves published loads unchanged");
    bt::require(extreme_tire->diagnostic_storage()[0].brush_loss_j ==
                        diag0_loss_before &&
                        extreme_tire->diagnostic_storage()[1].brush_loss_j ==
                                diag1_loss_before,
                "rejected cumulative overflow leaves published per-wheel losses unchanged");

    // (4) Retry the SAME rejected timestamp with one wheel airborne:
    // success proves the rejected step did not advance the interval clock.
    set_front(false);
    set_rear(true);
    bt::require(extreme_tire->try_compute_into(d, dt, out) == CoreStatus::ok,
                "retry at the SAME rejected timestamp succeeds single-wheel (clock not advanced)");
    bt::require(std::isfinite(extreme_tire->brush_loss_step_j()),
                "same-timestamp retry commits a finite single-wheel loss");
}

} // namespace

int main() {
    return bt::run([] {
        test_stage_a_shear_stiffness_scaling_standstill_and_rolling();
        test_stage_a_transition_energy_conservation_and_wire_roundtrip();
        test_stage_a_wheel_load_sensitivity_and_monotonic_traction();
        test_stage_a_world_rolling_moment_no_caliper_reaction_and_reset_capacity();
        test_stage_a_rolling_crr_overflow_rejected_and_state_atomic();
        test_stage_a_two_wheel_cumulative_loss_overflow_rejected_and_state_atomic();
        test_signed_distance_matches_python_bitwise();
        test_config_projection_carries_the_distributed_section();
        test_config_rejects_bad_distributed_sections();
        test_distributed_state_wire_round_trip();
        test_distributed_qfrc_bitwise();
        test_distributed_probe_and_clock();
        test_distributed_observations_declare_multi_contact();
    });
}
