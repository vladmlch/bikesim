// Completion 3.20/T52: C++ port of tests/reference/test_native_suspension.py.
//
// test_suspension_components_bitwise replays the frozen post-F2 pinned
// episode: per recorded state row the Stepper is restored and forwarded,
// then the writer's component dict must equal the artifact's recorded
// acc.add rows bitwise, in the golden force-matrix's insertion order
// (shock_hbo exists only in physical mode).
// test_suspension_components_bitwise_legacy replays the frozen legacy_env
// bundle (<platform>/native_suspension/legacy_env/): RideSimulation()'s
// compiled model + projected wire + baseline qpos/resolved joint
// addresses; the probe matrix keeps the 7-component dict reachable.
#include "tyre_reference_support.hpp"

#include "io/json.hpp"
#include "model_access.hpp"
#include "runtime/setup_values.hpp"

namespace {

using namespace tyre_reference;
namespace io = bike::io;
namespace setupvalue = runtime::setupvalue;

constexpr std::string_view kFile = "test_native_suspension.py";

[[nodiscard]] std::string mjb() {
    return repo_file("tests/fixtures/benchmark/model.mjb")
            .string();
}

[[gnu::noinline]] void test_suspension_components_bitwise() {
    auto golden = node(kFile, "test_suspension_components_bitwise");
    const auto bundle =
            platform_fixture("episode", "pinned_savage_40_f2");
    const auto ep = load_episode_bundle(bundle);
    Stepper st((bundle / "model.mjb").string(), load_config(bundle));
    std::vector<std::string> sus_names;
    for (const auto &name : ep.artifact.force_names)
        if (name.starts_with("fork_") || name.starts_with("shock_"))
            sus_names.push_back(name);
    const std::size_t steps = ep.artifact.state_qpos.shape.at(0);
    for (std::size_t k = 0; k < steps; ++k) {
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na), row(ep.warmstart, k, ep.nv),
                     ep.time.at(k));
        st.forward();
        const auto comp = st.suspension_components();
        bt::require(comp.size() == sus_names.size(),
                    "native dict carries exactly the suspension names");
        for (std::size_t i = 0; i < comp.size(); ++i)
            bt::require(comp[i].first == sus_names[i],
                        "suspension components keep the golden order");
        for (const auto &name : sus_names) {
            const auto it = std::ranges::find_if(
                    comp, [&](const auto &entry) {
                        return entry.first == name;
                    });
            golden.f64(it->second,
                       std::array<std::size_t, 1>{it->second.size()},
                       name + " step " + std::to_string(k));
        }
    }
    golden.done();
}

[[gnu::noinline]] void test_suspension_components_bitwise_legacy() {
    auto golden =
            node(kFile, "test_suspension_components_bitwise_legacy");
    const auto bundle =
            platform_fixture("native_suspension", "legacy_env");
    const auto bytes = fixture_bytes(bundle / "model.mjb");
    const auto meta =
            io::parse_json(fixture_text(bundle / "meta.json"));
    const auto read_int = [&](std::string_view key) {
        return io::number_i64(
                io::as_number(
                        *io::find_member(io::as_object(meta, "meta"), key),
                        "meta"),
                "meta." + std::string(key));
    };
    const std::size_t nq = static_cast<std::size_t>(read_int("nq"));
    const std::size_t nv = static_cast<std::size_t>(read_int("nv"));
    const std::size_t na = static_cast<std::size_t>(read_int("na"));
    const std::size_t fork_q =
            static_cast<std::size_t>(read_int("fork_qposadr"));
    const std::size_t shock_q =
            static_cast<std::size_t>(read_int("shock_qposadr"));
    const std::size_t fork_v_adr =
            static_cast<std::size_t>(read_int("fork_dofadr"));
    const std::size_t shock_v_adr =
            static_cast<std::size_t>(read_int("shock_dofadr"));
    const auto &qpos0_json = io::as_array(
            *io::find_member(io::as_object(meta, "meta"), "qpos0"),
            "meta.qpos0");
    bt::require(qpos0_json.size() == nq, "legacy_env qpos0 width");
    std::vector<double> qpos0;
    for (const auto &cell : qpos0_json)
        qpos0.push_back(io::as_double(cell, "meta.qpos0"));
    Stepper st(bytes, load_config(bundle));
    const std::array expected{"fork_spring", "fork_damper", "shock_coil",
                              "shock_bumper", "shock_damper",
                              "shock_top_out", "shock_upper_stop"};
    struct Probe {
        double travel_mm, stroke_mm, fork_v, shock_v;
        const char *travel_s, *stroke_s;
    };
    const std::array probes{Probe{.travel_mm = 0.,
                                  .stroke_mm = 0.,
                                  .fork_v = 0.,
                                  .shock_v = 0.,
                                  .travel_s = "0.0",
                                  .stroke_s = "0.0"},
                            Probe{.travel_mm = 61.3,
                                  .stroke_mm = 12.5,
                                  .fork_v = 0.0087,
                                  .shock_v = -0.31,
                                  .travel_s = "61.3",
                                  .stroke_s = "12.5"},
                            Probe{.travel_mm = 179.5,
                                  .stroke_mm = 64.9,
                                  .fork_v = 2.4,
                                  .shock_v = 1.9,
                                  .travel_s = "179.5",
                                  .stroke_s = "64.9"},
                            Probe{.travel_mm = 0.5,
                                  .stroke_mm = 1.,
                                  .fork_v = -1.2,
                                  .shock_v = -0.9,
                                  .travel_s = "0.5",
                                  .stroke_s = "1.0"}};
    const std::vector<double> act(na, 0.);
    for (const auto &probe : probes) {
        std::vector<double> qpos = qpos0;
        std::vector<double> qvel(nv, 0.);
        std::vector<double> warmstart(nv, 0.);
        qpos[fork_q] = probe.travel_mm / 1000.;
        qpos[shock_q] = probe.stroke_mm / 1000.;
        qvel[fork_v_adr] = probe.fork_v;
        qvel[shock_v_adr] = probe.shock_v;
        st.set_state(qpos, qvel, act, warmstart, 0.);
        st.forward();
        const auto comp = st.suspension_components();
        bt::require(comp.size() == expected.size(),
                    "legacy mode emits the seven-component dict");
        const std::string tag = " @ travel=" +
                std::string(probe.travel_s) + " stroke=" +
                std::string(probe.stroke_s);
        for (std::size_t i = 0; i < comp.size(); ++i) {
            bt::require(comp[i].first == expected[i],
                        "legacy component order");
            golden.f64(comp[i].second,
                       std::array<std::size_t, 1>{comp[i].second.size()},
                       comp[i].first + tag);
        }
    }
    golden.done();
}

[[gnu::noinline]] void test_suspension_components_require_config() {
    Stepper plain(mjb());
    Stepper empty_cfg(mjb(), nativecfg::WriterConfig{});
    for (Stepper *st : {&plain, &empty_cfg})
        rejects<std::logic_error>(
                [&] { (void)st->suspension_components(); });
}

[[gnu::noinline]] void test_config_missing_key_names_it() {
    rejects<std::invalid_argument>(
            [] {
                io::Value::Object root;
                root.emplace_back("suspension",
                                  io::Value{io::Value::Object{}});
                Stepper const st(mjb(),
                                 nativecfg::decode(io::Value{std::move(root)}));
            },
            "schema");
    rejects<std::invalid_argument>(
            [] {
                io::Value::Object root;
                root.emplace_back("schema", setupvalue::integer(1));
                root.emplace_back("suspension",
                                  io::Value{io::Value::Object{}});
                Stepper const st(mjb(),
                                 nativecfg::decode(io::Value{std::move(root)}));
            },
            "physics_mode");
}

} // namespace

int main() {
    return bt::run([] {
        test_suspension_components_bitwise();
        test_suspension_components_bitwise_legacy();
        test_suspension_components_require_config();
        test_config_missing_key_names_it();
    });
}
