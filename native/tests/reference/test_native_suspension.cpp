// Completion 3.20/T52: C++ port of tests/reference/test_native_suspension.py.
//
// test_suspension_components_bitwise replays the frozen post-F2 pinned
// episode: per recorded state row the Stepper is restored and forwarded,
// then the writer's component dict must equal the artifact's recorded
// acc.add rows bitwise, in the golden force-matrix's insertion order.
// The legacy_env replay test left with the legacy damper curve
// (2026-11-04): the wire rejects it at decode now, so the 7-component
// dict has no remaining producer.
#include "tyre_reference_support.hpp"

#include "io/json.hpp"
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
        test_suspension_components_require_config();
        test_config_missing_key_names_it();
    });
}
