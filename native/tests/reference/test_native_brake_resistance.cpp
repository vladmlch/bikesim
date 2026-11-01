// Completion 3.20/T52: C++ port of
// tests/reference/test_native_brake_resistance.py.
//
// The two golden consumers replay the frozen post-F2 pinned episode
// (tests/fixtures/golden/<platform>/episode/pinned_savage_40_f2/): per
// recorded state row the Stepper is restored and forwarded, then the
// demand sweep consumes 'brake_torques(...)'/ 'apply_brake(...)' vectors
// and the resistance writer consumes the manifest's tire-snapshot rows,
// compared bitwise against both recorded leaves ('... oracle step k' =
// the live-Python component vector, '... golden step k' = the artifact's
// force matrix row — the capture-time dual run proved them identical).
// test_resistance_snapshot_array_conversion_lifetime is a binding-level
// ndarray lifetime contract with no TireSideInput-span analog — a
// recorded drop in the ledger.
#include "tyre_reference_support.hpp"

#include "model_access.hpp"
#include "runtime/setup_values.hpp"
#include "writers/resistance.hpp"

#include <algorithm>
#include <limits>

namespace {

using namespace tyre_reference;
namespace io = bike::io;
namespace setupvalue = runtime::setupvalue;

constexpr std::string_view kFile = "test_native_brake_resistance.py";
constexpr std::array kDemands{0., .3, .7, 1., -.2, 1.5};
// str(float) spellings the Python f-string labels captured.
constexpr std::array kDemandNames{"0.0", "0.3", "0.7", "1.0", "-0.2",
                                  "1.5"};

[[nodiscard]] std::string mjb() {
    return repo_file("tools/proto_native_bench/artifacts/model.mjb")
            .string();
}

[[nodiscard]] const io::Value &member(const io::Value &object,
                                      std::string_view key, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                                      std::string_view context) {
    const auto &fields = io::as_object(object, context);
    for (const auto &[name, value] : fields)
        if (name == key) return value;
    bt::require(false, "episode manifest member missing: " +
                               std::string(key));
    std::abort();
}

// Flattened tire-snapshot row -> binding-shaped TireSideInput spans. The
// vectors stay owned by `Row` for the duration of the writer call —
// lifetime is the test's job now that no ndarray adapter sits between.
struct Side {
    std::vector<double> loads;
    std::vector<bool> working_bits;
    std::unique_ptr<bool[]> working; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    double radius = 0.;
    [[nodiscard]] TireSideInput input() const {
        return TireSideInput{.patch_loads = loads,
                             .patch_working = {working.get(),
                                              working_bits.size()},
                             .effective_radius_m = radius};
    }
};
[[nodiscard]] Side side_input(const io::Value &snap,
                              std::string_view context) {
    Side side;
    side.radius = io::as_double(member(snap, "effective_radius_m", context),
                                context);
    const auto &patches =
            io::as_array(member(snap, "patches", context), context);
    side.working = std::make_unique<bool[]>(patches.size()); // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    for (const auto &patch : patches) {
        side.loads.push_back(io::as_double(
                member(patch, "normal_load_n", context), context));
        side.working_bits.push_back(io::as_bool(
                member(patch, "working_surface", context), context));
    }
    std::ranges::copy(side.working_bits, side.working.get());
    return side;
}

[[gnu::noinline]] void test_brake_torques_and_apply_bitwise() {
    auto golden =
            node(kFile, "test_brake_torques_and_apply_bitwise");
    const auto bundle =
            platform_fixture("episode", "pinned_savage_40_f2");
    const auto ep = load_episode_bundle(bundle);
    Stepper st((bundle / "model.mjb").string(), load_config(bundle));
    const int front_adr = mj_name2id(st.model(), mjOBJ_ACTUATOR,
                                     "front_brake");
    const int rear_adr =
            mj_name2id(st.model(), mjOBJ_ACTUATOR, "rear_brake");
    model_access::require_id(front_adr, st.model()->nu, "front_brake");
    model_access::require_id(rear_adr, st.model()->nu, "rear_brake");
    const auto ctrl_rows = ep.artifact.ctrl_written.doubles(true);
    const std::size_t nu = static_cast<std::size_t>(st.model()->nu);
    const std::size_t steps = ep.artifact.state_qpos.shape.at(0);
    for (std::size_t k = 0; k < steps; ++k) {
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na), row(ep.warmstart, k, ep.nv),
                     ep.time.at(k));
        st.forward();
        const auto ctrl = row(ctrl_rows, k, nu);
        bt::require(ctrl[static_cast<std::size_t>(front_adr)] == 0. &&
                            ctrl[static_cast<std::size_t>(rear_adr)] == 0.,
                    "episode captured zeroed brake channels");
        for (std::size_t fi = 0; fi < kDemands.size(); ++fi)
            for (std::size_t ri = 0; ri < kDemands.size(); ++ri) {
                const std::string tag =
                        "(" + std::string(kDemandNames[fi]) + "," +
                        std::string(kDemandNames[ri]) + ") step " +
                        std::to_string(k);
                const auto [nf, nr] = st.brake_torques(kDemands[fi],
                                                       kDemands[ri]);
                const std::array pair{nf, nr};
                golden.f64(pair, std::array<std::size_t, 1>{2},
                           "brake_torques" + tag);
                st.apply_brake(kDemands[fi], kDemands[ri]);
                golden.f64(st.ctrl(), std::array<std::size_t, 1>{nu},
                           "apply_brake" + tag);
            }
    }
    golden.done();
}

[[gnu::noinline]] void test_resistance_components_bitwise() {
    auto golden =
            node(kFile, "test_resistance_components_bitwise");
    const auto bundle =
            platform_fixture("episode", "pinned_savage_40_f2");
    const auto ep = load_episode_bundle(bundle);
    Stepper st((bundle / "model.mjb").string(), load_config(bundle));
    const auto &snapshots = io::as_array(
            member(ep.artifact.manifest, "tire_snapshots", "manifest"),
            "manifest.tire_snapshots");
    const std::size_t steps = ep.artifact.state_qpos.shape.at(0);
    for (std::size_t k = 0; k < steps; ++k) {
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na), row(ep.warmstart, k, ep.nv),
                     ep.time.at(k));
        st.forward();
        const std::string context =
                "manifest.tire_snapshots[" + std::to_string(k) + "]";
        const auto front =
                side_input(member(snapshots.at(k), "front", context),
                           context + ".front");
        const auto rear =
                side_input(member(snapshots.at(k), "rear", context),
                           context + ".rear");
        const auto components =
                st.resistance_components(front.input(), rear.input());
        bt::require(components.size() == 2 &&
                            components[0].first == "road_rolling" &&
                            components[1].first == "aerodynamic",
                    "resistance components keep the dict's order");
        for (const auto &[name, values] : components) {
            golden.f64(values,
                       std::array<std::size_t, 1>{values.size()},
                       name + " oracle step " + std::to_string(k));
            golden.f64(values,
                       std::array<std::size_t, 1>{values.size()},
                       name + " golden step " + std::to_string(k));
        }
    }
    golden.done();
}

[[gnu::noinline]] void test_brake_resistance_require_config() {
    const std::vector<double> no_loads;
    const bool no_working[1] = {false}; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
    const TireSideInput empty{.patch_loads = no_loads,
                              .patch_working = {no_working, 0}, // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
                              .effective_radius_m = .3};
    Stepper plain(mjb());
    Stepper empty_cfg(mjb(), nativecfg::WriterConfig{});
    for (Stepper *st : {&plain, &empty_cfg}) {
        rejects<std::logic_error>(
                [&] { (void)st->brake_torques(0., 0.); });
        rejects<std::logic_error>([&] { st->apply_brake(0., 0.); });
        rejects<std::logic_error>([&] {
            (void)st->resistance_components(empty, empty);
        });
        // The malformed-dict early rejection
        // (resistance_components({}) -> ValueError 'front') is binding
        // input marshalling — no TireSideInput-span analog exists.
    }
}

[[gnu::noinline]] void test_brake_resistance_missing_key_names_it() {
    rejects<std::invalid_argument>(
            [] {
                io::Value::Object root;
                root.emplace_back("schema", setupvalue::integer(1));
                root.emplace_back("brake", io::Value{io::Value::Object{}});
                Stepper const st(mjb(),
                                 nativecfg::decode(io::Value{std::move(root)}));
            },
            "torque_ceiling_nm");
    rejects<std::invalid_argument>(
            [] {
                io::Value::Object root;
                root.emplace_back("schema", setupvalue::integer(1));
                root.emplace_back("resistance",
                                  io::Value{io::Value::Object{}});
                Stepper const st(mjb(),
                                 nativecfg::decode(io::Value{std::move(root)}));
            },
            "crr");
}

} // namespace

int main() {
    return bt::run([] {
        test_brake_torques_and_apply_bitwise();
        test_resistance_components_bitwise();
        test_brake_resistance_require_config();
        test_brake_resistance_missing_key_names_it();
    });
}
