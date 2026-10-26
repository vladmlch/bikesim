// Completion 3.20/T52: C++ port of tests/reference/test_native_cruise.py.
//
// The sequential scalar PI oracle is replayed against the recorded
// two-platform goldens: the 15 literal head cases verbatim plus the
// 500-case tail through bike::random::NumpyRng(728), the PCG64 mirror of
// np.random.default_rng(728). Each case consumes the recorded scalar
// output entry and the 4-vector numeric state entry in capture order;
// the engaged flag is asserted against the oracle's rule
// controller_grounded.value_or(rear_in_contact). Config/state validation
// is ported at the typed nativecfg::Value decoder and CruiseState faces;
// the nanobind dict-shape halves of
// test_missing_state_field_preserves_state,
// test_extra_state_field_rejected,
// test_nonnumeric_restore_preserves_state and the two ('engaged', ...)
// params of test_invalid_restore_preserves_state have no core seam and
// are recorded as binding-contract drops in the ledger.
#include "tyre_reference_support.hpp"

#include "configuration/fields.hpp"
#include "initialization/capture.hpp"
#include "random/numpy_compat.hpp"
#include "runtime/setup_values.hpp"
#include "support/model_fixture.hpp"
#include "writers/cruise.hpp"

#include <cmath>
#include <limits>
#include <optional>

namespace {

using namespace tyre_reference;
namespace io = bike::io;
namespace setupvalue = runtime::setupvalue;

constexpr std::string_view kFile = "test_native_cruise.py";
constexpr std::string_view kXml =
        R"(<mujoco><option timestep="0.0005"/><worldbody><body>
  <joint name="root_x" type="slide" axis="1 0 0"/><geom type="sphere" size=".1" mass="1"/>
</body></worldbody></mujoco>)";
constexpr nativecfg::CruiseConfig kConfig{
        .target_speed_kmh = 25., .kp_nm_per_mps = 180.,
        .ki_nm_per_mps_s = 150., .torque_ceiling_nm = 150.};
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kMax = std::numeric_limits<double>::max();

// (speed, rear_in_contact, traction_limited, controller_grounded).
struct Case {
    double speed;
    bool grounded, limited;
    std::optional<bool> override_;
};

// The Python `pair` fixture: the compiled slide plant, fresh MjData-value
// buffers (qpos/qvel/act/qacc_warmstart all zero, time 0) and the Stepper
// over the saved image with {'schema': 1, 'cruise': CONFIG}.
struct Pair {
    explicit Pair(nativecfg::CruiseConfig config = kConfig,
                  std::string_view xml = kXml)
        : fixture(xml) {
        const auto image = bike::initialization::capture_model_image(
                *fixture.model);
        auto cfg = std::make_unique<nativecfg::WriterConfig>();
        cfg->cruise = config;
        native = std::make_unique<Stepper>(image.mjb, *cfg);
        qpos.assign(native->qpos().begin(), native->qpos().end());
        qvel.assign(native->qvel().begin(), native->qvel().end());
        act.assign(static_cast<std::size_t>(native->model()->na), 0.);
        warmstart.assign(static_cast<std::size_t>(native->model()->nv),
                         0.);
    }
    bt::ModelFixture fixture;
    std::unique_ptr<Stepper> native;
    std::vector<double> qpos, qvel, act, warmstart;
    double time = 0.;
};

// _state(oracle): NUMERIC order; the engaged flag equals the oracle's
// post-compute flag — engaged iff override.value_or(grounded) on the last
// call (the gated branch stores engaged=false).
void assert_state(bt::oracle::Node &golden, const Stepper &st,
                  bool expected_engaged) {
    const auto state = st.cruise_state();
    bt::require(state.engaged == expected_engaged,
                "engaged flag mirrors the oracle's post-compute state");
    const std::array values{state.target_speed_mps, state.integral_mps_s,
                            state.torque_nm, state.gain_scale};
    golden.f64(values, std::array<std::size_t, 1>{4}, "");
}

[[nodiscard]] std::vector<double>
run(bt::oracle::Node &golden, Pair &pair, std::span<const Case> cases) {
    std::vector<double> outputs;
    outputs.reserve(cases.size());
    for (const auto &c : cases) {
        pair.qvel[0] = c.speed;
        pair.native->set_state(pair.qpos, pair.qvel, pair.act,
                               pair.warmstart, pair.time);
        const std::array out{pair.native->cruise_compute(
                c.grounded, c.limited, c.override_)};
        golden.f64(out, std::span<const std::size_t>{}, "");
        assert_state(golden, *pair.native,
                     c.override_.value_or(c.grounded));
        outputs.push_back(out.front());
    }
    return outputs;
}

[[nodiscard]] bool same_state(const CruiseState &a, const CruiseState &b) {
    return std::bit_cast<std::uint64_t>(a.target_speed_mps) ==
                    std::bit_cast<std::uint64_t>(b.target_speed_mps) &&
            std::bit_cast<std::uint64_t>(a.integral_mps_s) ==
                    std::bit_cast<std::uint64_t>(b.integral_mps_s) &&
            std::bit_cast<std::uint64_t>(a.torque_nm) ==
                    std::bit_cast<std::uint64_t>(b.torque_nm) &&
            a.engaged == b.engaged &&
            std::bit_cast<std::uint64_t>(a.gain_scale) ==
                    std::bit_cast<std::uint64_t>(b.gain_scale);
}
void require_preserved(const Stepper &st, const CruiseState &old) {
    bt::require(same_state(st.cruise_state(), old),
                "rejected call preserves the cruise state bitwise");
}

// ---------------------------------------------------------------------
// Golden consumers — the recorded entry stream per source node.
// ---------------------------------------------------------------------

[[gnu::noinline]] void test_sequential_bitwise_oracle() {
    auto golden = node(kFile, "test_sequential_bitwise_oracle");
    Pair pair;
    const double t = pair.native->cruise_state().target_speed_mps;
    std::vector<Case> cases;
    for (const double speed : {-0., 0., 1., t - .4, t - .1, t, t + .1,
                               t + .4, 12., 20.})
        cases.push_back(Case{.speed = speed, .grounded = true,
                             .limited = false,
                             .override_ = std::nullopt});
    cases.push_back(Case{.speed = t - .2, .grounded = false,
                         .limited = false, .override_ = std::nullopt});
    cases.push_back(Case{.speed = t + .2, .grounded = false,
                         .limited = true, .override_ = true});
    cases.push_back(Case{.speed = t - .2, .grounded = true,
                         .limited = false, .override_ = false});
    cases.push_back(Case{.speed = t + .2, .grounded = true,
                         .limited = true, .override_ = std::nullopt});
    cases.push_back(Case{.speed = t - .2, .grounded = true,
                         .limited = false, .override_ = std::nullopt});
    // np.random.default_rng(728): uniform(3,12), integers(2),
    // integers(2), integers(3) -> (None, False, True) index.
    bike::random::NumpyRng rng(728);
    static const std::array<std::optional<bool>, 3> overrides = {
            std::nullopt, false, true};
    for (int i = 0; i < 500; ++i)
        cases.push_back(Case{
                .speed = rng.uniform(3., 12.),
                .grounded = rng.integers(2) != 0,
                .limited = rng.integers(2) != 0,
                .override_ = overrides.at(
                        static_cast<size_t>(rng.integers(3)))});
    (void)run(golden, pair, cases);
    golden.done();
}

[[gnu::noinline]] void restored_integral(std::string_view id,
                                         double integral) {
    auto golden =
            node(kFile,
                 "test_restored_integral_unwinds_and_exact_saturation",
                 id);
    Pair pair;
    CruiseState state = pair.native->cruise_state();
    state.integral_mps_s = integral;
    pair.native->set_cruise_state(state);
    const double t = pair.native->cruise_state().target_speed_mps;
    const std::array cases{
            Case{.speed = t, .grounded = true, .limited = false,
                 .override_ = std::nullopt},
            Case{.speed = t - .1, .grounded = true, .limited = false,
                 .override_ = std::nullopt},
            Case{.speed = t + .1, .grounded = true, .limited = false,
                 .override_ = std::nullopt},
            Case{.speed = t, .grounded = false, .limited = false,
                 .override_ = std::nullopt}};
    (void)run(golden, pair, cases);
    golden.done();
}

[[gnu::noinline]] void test_signed_zero_snapshot_and_compute() {
    auto golden = node(kFile, "test_signed_zero_snapshot_and_compute");
    Pair pair;
    CruiseState state = pair.native->cruise_state();
    state.integral_mps_s = -0.;
    state.torque_nm = -0.;
    pair.native->set_cruise_state(state);
    assert_state(golden, *pair.native, false);
    const std::array cases{
            Case{.speed = state.target_speed_mps, .grounded = true,
                 .limited = true, .override_ = std::nullopt},
            Case{.speed = -0., .grounded = false, .limited = false,
                 .override_ = std::nullopt}};
    (void)run(golden, pair, cases);
    golden.done();
}

[[gnu::noinline]] void test_target_compensation_reset_and_defaults() {
    auto golden =
            node(kFile, "test_target_compensation_reset_and_defaults");
    Pair pair;
    for (const double target : {15., 25., 45.}) {
        pair.native->cruise_set_target_speed(target);
        for (const double support :
             {-12., -0., 0., .3, 2., 5., 1e300, kMax}) {
            const std::array scale{
                    pair.native->cruise_set_assist_compensation(support)};
            golden.f64(scale, std::span<const std::size_t>{}, "");
            const std::array cases{Case{
                    .speed = pair.native->cruise_state()
                                             .target_speed_mps -
                                    .2,
                    .grounded = true, .limited = false,
                    .override_ = std::nullopt}};
            (void)run(golden, pair, cases);
        }
    }
    pair.native->cruise_reset();
    assert_state(golden, *pair.native, false);
    // Post-reset compute on the unchanged mjData — qvel still holds the
    // last case's speed because set_state was not re-run.
    const std::array out{pair.native->cruise_compute(true, false,
                                                     std::nullopt)};
    golden.f64(out, std::span<const std::size_t>{}, "");
    golden.done();
}

[[gnu::noinline]] void
test_snapshot_replay_and_mjdata_restore_is_independent() {
    auto golden = node(kFile,
                       "test_snapshot_replay_and_mjdata_restore_is_"
                       "independent");
    Pair pair;
    const double t = pair.native->cruise_state().target_speed_mps;
    const std::array warm_case{Case{.speed = t - .2, .grounded = true,
                                    .limited = false,
                                    .override_ = std::nullopt}};
    for (int i = 0; i < 10; ++i) (void)run(golden, pair, warm_case);
    const CruiseState snapshot = pair.native->cruise_state();
    pair.native->set_state(pair.qpos, pair.qvel, pair.act, pair.warmstart,
                           12.);
    // mjData restore leaves the PI state untouched; engaged survived as
    // true from the grounded head cases.
    assert_state(golden, *pair.native, true);
    std::vector<Case> cases;
    for (int i = 0; i < 20; ++i) {
        cases.push_back(Case{.speed = t + .3, .grounded = true,
                             .limited = false, .override_ = std::nullopt});
        cases.push_back(Case{.speed = t, .grounded = false,
                             .limited = false, .override_ = true});
    }
    (void)run(golden, pair, cases);
    pair.native->set_cruise_state(snapshot);
    const auto second = run(golden, pair, cases);
    const std::array<std::size_t, 1> shape{second.size()};
    golden.f64(second, shape, "");
    golden.done();
    // Dict/struct independence: mutating the snapshot copy cannot reach
    // the writer's state.
    CruiseState detached = snapshot;
    detached.integral_mps_s = 999.;
    bt::require(pair.native->cruise_state().integral_mps_s != 999.,
                "cruise_state() returns a detached snapshot");
}

[[gnu::noinline]] void exact_saturation(std::string_view id, double sign) {
    auto golden = node(kFile, "test_exact_saturation_freezes_nonzero_error",
                       id);
    Pair pair;
    CruiseState state = pair.native->cruise_state();
    state.integral_mps_s = sign * .4;
    pair.native->set_cruise_state(state);
    const std::array cases{Case{
            .speed = pair.native->cruise_state().target_speed_mps -
                            sign * .5,
            .grounded = true, .limited = false,
            .override_ = std::nullopt}};
    (void)run(golden, pair, cases);
    golden.done();
    const auto after = pair.native->cruise_state();
    bt::require(after.integral_mps_s == sign * .4,
                "saturated-in-error-direction freezes the integrator");
    bt::require(after.torque_nm == sign * 150.,
                "demand saturates at the ceiling");
}

[[gnu::noinline]] void test_negative_zero_output_bitwise() {
    auto golden = node(kFile, "test_negative_zero_output_bitwise");
    constexpr nativecfg::CruiseConfig config{
            .target_speed_kmh = 25., .kp_nm_per_mps = 1e-300,
            .ki_nm_per_mps_s = 1e-300, .torque_ceiling_nm = 150.};
    Pair pair(config);
    CruiseState state = pair.native->cruise_state();
    state.gain_scale = 1e-20;
    state.integral_mps_s = -0.;
    pair.native->set_cruise_state(state);
    const std::array cases{Case{
            .speed = pair.native->cruise_state().target_speed_mps + 1e-6,
            .grounded = true, .limited = true, .override_ = std::nullopt}};
    const auto outputs = run(golden, pair, cases);
    const std::array<std::size_t, 1> shape{outputs.size()};
    golden.f64(outputs, shape, "");
    golden.done();
}

[[gnu::noinline]] void restored_target(std::string_view id, double target) {
    auto golden = node(kFile, "test_restored_target_keeps_mps_bits", id);
    Pair pair;
    CruiseState state = pair.native->cruise_state();
    state.target_speed_mps = target;
    pair.native->set_cruise_state(state);
    // The oracle's engaged flag is still false — no compute ran.
    assert_state(golden, *pair.native, false);
    const std::array cases{Case{.speed = target - .2, .grounded = true,
                                .limited = false,
                                .override_ = std::nullopt}};
    (void)run(golden, pair, cases);
    golden.done();
}

// ---------------------------------------------------------------------
// Contract ports — config decode and state validation.
// ---------------------------------------------------------------------

// A finite Number lexeme keeps CPython float() repr spelling; the
// nonfinite sentinels exercise finite_real's rejection path.
[[nodiscard]] io::Value num(double value) {
    if (std::isnan(value)) return io::Value{io::Number{"nan"}};
    if (!std::isfinite(value))
        return io::Value{io::Number{std::signbit(value) ? "-inf" : "inf"}};
    return setupvalue::real(value);
}
[[nodiscard]] io::Value integer(long long value) {
    return setupvalue::integer(value);
}

constexpr std::array kCruiseFields{
        std::pair{"target_speed_kmh", 25.}, std::pair{"kp_nm_per_mps", 180.},
        std::pair{"ki_nm_per_mps_s", 150.},
        std::pair{"torque_ceiling_nm", 150.}};
[[nodiscard]] io::Value cruise_section(
        std::string_view drop = {},
        std::optional<std::pair<std::string_view, double>> replace = {}) {
    io::Value::Object object;
    for (const auto &[name, value] : kCruiseFields) {
        const std::string key(name);
        if (key == drop) continue;
        object.emplace_back(key,
                            num(replace && replace->first == name
                                        ? replace->second
                                        : value));
    }
    return io::Value{std::move(object)};
}
[[nodiscard]] nativecfg::WriterConfig wire(const io::Value &section) {
    io::Value::Object root;
    root.emplace_back("schema", integer(1));
    root.emplace_back("cruise", section);
    return nativecfg::decode(io::Value{std::move(root)});
}
[[nodiscard]] io::Value::Object document(
        std::initializer_list<std::pair<std::string, io::Value>> fields) {
    io::Value::Object root;
    for (auto &[name, value] : fields)
        root.emplace_back(std::move(name), std::move(value));
    return root;
}

[[nodiscard]] std::vector<std::byte> image(std::string_view xml = kXml) {
    bt::ModelFixture fixture(xml);
    return bike::initialization::capture_model_image(*fixture.model).mjb;
}

[[gnu::noinline]] void every_api_requires_config(
        const io::Value *config) {
    const auto bytes = image();
    const auto st = config ? std::make_unique<Stepper>(
                                     bytes, nativecfg::decode(*config))
                           : std::make_unique<Stepper>(bytes);
    rejects<std::logic_error>(
            [&] { (void)st->cruise_compute(true, false, {}); }, "cruise");
    rejects<std::logic_error>([&] { st->cruise_reset(); }, "cruise");
    rejects<std::logic_error>(
            [&] { st->cruise_set_target_speed(25.); }, "cruise");
    rejects<std::logic_error>(
            [&] { (void)st->cruise_set_assist_compensation(0.); }, "cruise");
    rejects<std::logic_error>([&] { (void)st->cruise_state(); }, "cruise");
    rejects<std::logic_error>(
            [&] { st->set_cruise_state(CruiseState{}); }, "cruise");
}

[[gnu::noinline]] void missing_config_key_names_it(std::string_view key) {
    const auto bytes = image();
    rejects<std::invalid_argument>(
            [&] { Stepper st(bytes, wire(cruise_section(key))); }, key);
}

[[gnu::noinline]] void schema_and_section_validation(io::Value document) {
    const auto bytes = image();
    rejects<std::invalid_argument>(
            [&] { Stepper st(bytes, nativecfg::decode(document)); });
}

[[gnu::noinline]] void invalid_config_value(std::string_view key,
                                            double value) {
    const auto bytes = image();
    rejects<std::invalid_argument>(
            [&] {
                Stepper st(bytes,
                           wire(cruise_section(
                                   {}, std::pair<std::string_view, double>{
                                               key, value})));
            },
            key);
}

[[gnu::noinline]] void config_target_band(double target) {
    const auto bytes = image();
    rejects<std::invalid_argument>([&] {
        Stepper st(bytes,
                   wire(cruise_section(
                           {}, std::pair<std::string_view, double>{
                                       "target_speed_kmh", target})));
    });
}

[[gnu::noinline]] void root_joint_validation(std::string_view name,
                                             std::string_view kind) {
    const std::string xml =
            "<mujoco><option timestep=\"0.0005\"/><worldbody><body>"
            "<joint name=\"" + std::string(name) + "\" type=\"" +
            std::string(kind) +
            "\" axis=\"1 0 0\"/>"
            "<geom type=\"sphere\" size=\".1\" mass=\"1\"/>"
            "</body></worldbody></mujoco>";
    const auto bytes = image(xml);
    auto cfg = std::make_unique<nativecfg::WriterConfig>();
    cfg->cruise = kConfig;
    rejects<std::invalid_argument>([&] { Stepper st(bytes, *cfg); },
                                   "root_x");
}

[[gnu::noinline]] void nonfinite_setters_preserve_state(
        std::string_view method, double value) {
    Pair pair;
    const auto old = pair.native->cruise_state();
    if (method == "cruise_set_target_speed")
        rejects<std::invalid_argument>(
                [&] { pair.native->cruise_set_target_speed(value); });
    else
        rejects<std::invalid_argument>([&] {
            (void)pair.native->cruise_set_assist_compensation(value);
        });
    require_preserved(*pair.native, old);
}

[[gnu::noinline]] void target_setter_band_preserves_state(double value) {
    Pair pair;
    const auto old = pair.native->cruise_state();
    rejects<std::invalid_argument>(
            [&] { pair.native->cruise_set_target_speed(value); });
    require_preserved(*pair.native, old);
}

[[gnu::noinline]] void nonfinite_restore_preserves_state(
        std::string_view key, double value) {
    Pair pair;
    const auto old = pair.native->cruise_state();
    auto bad = old;
    if (key == "target_speed_mps")
        bad.target_speed_mps = value;
    else if (key == "integral_mps_s")
        bad.integral_mps_s = value;
    else if (key == "torque_nm")
        bad.torque_nm = value;
    else
        bad.gain_scale = value;
    rejects<std::invalid_argument>(
            [&] { pair.native->set_cruise_state(bad); }, key);
    require_preserved(*pair.native, old);
}

[[gnu::noinline]] void invalid_restore_preserves_state() {
    Pair pair;
    const auto old = pair.native->cruise_state();
    for (const double target : {15. / 3.6 - .001, 45. / 3.6 + .001}) {
        auto bad = old;
        bad.target_speed_mps = target;
        rejects<std::invalid_argument>(
                [&] { pair.native->set_cruise_state(bad); });
        require_preserved(*pair.native, old);
    }
    for (const double gain : {0., -1., kMax}) {
        auto bad = old;
        bad.gain_scale = gain;
        rejects<std::invalid_argument>(
                [&] { pair.native->set_cruise_state(bad); });
        require_preserved(*pair.native, old);
    }
    // ('engaged', 1) and ('engaged', 'yes') are nanobind dict-type
    // rejections with no typed CruiseState analog — the binding-contract
    // half is a recorded drop.
}

[[gnu::noinline]] void test_support_underflow_preserves_state() {
    constexpr nativecfg::CruiseConfig config{
            .target_speed_kmh = 25., .kp_nm_per_mps = 180.,
            .ki_nm_per_mps_s = 1e-300, .torque_ceiling_nm = 150.};
    Pair pair(config);
    const auto old = pair.native->cruise_state();
    rejects<std::invalid_argument>([&] {
        (void)pair.native->cruise_set_assist_compensation(kMax);
    });
    require_preserved(*pair.native, old);
    auto bad = old;
    bad.gain_scale = 1e-100;
    rejects<std::invalid_argument>(
            [&] { pair.native->set_cruise_state(bad); });
    require_preserved(*pair.native, old);
}

[[gnu::noinline]] void test_projection_optional_cruise() {
    // The project(env) dict assertions are Python-projection semantics;
    // the ported half is the wire contract they feed the Stepper: a
    // projected cruise section configures the writer with a zeroed
    // runtime state, and an absent section disables every cruise API.
    const auto bytes = image();
    auto cfg = std::make_unique<nativecfg::WriterConfig>();
    cfg->cruise = nativecfg::CruiseConfig{
            .target_speed_kmh = 45., .kp_nm_per_mps = 180.,
            .ki_nm_per_mps_s = 150., .torque_ceiling_nm = 150.};
    Stepper enabled(bytes, *cfg);
    bt::require(enabled.cruise_state().integral_mps_s == 0. &&
                        enabled.cruise_state().target_speed_mps ==
                                45. / 3.6,
                "projected cruise config starts with a zeroed integrator");
    Stepper disabled(bytes, nativecfg::WriterConfig{});
    rejects<std::logic_error>([&] { (void)disabled.cruise_state(); },
                              "cruise");
}

} // namespace

int main() {
    return bt::run([] {
        test_sequential_bitwise_oracle();
        for (const auto &[id, integral] : std::array{
                     std::pair{"-100.0", -100.}, std::pair{"-1.0", -1.},
                     std::pair{"-0.0", -0.}, std::pair{"0.0", 0.},
                     std::pair{"1.0", 1.}, std::pair{"100.0", 100.}})
            restored_integral(id, integral);
        test_signed_zero_snapshot_and_compute();
        test_target_compensation_reset_and_defaults();
        test_snapshot_replay_and_mjdata_restore_is_independent();
        exact_saturation("-1.0", -1.);
        exact_saturation("1.0", 1.);
        test_negative_zero_output_bitwise();
        restored_target("4.166666666666667", 15. / 3.6);
        restored_target("12.5", 45. / 3.6);
        restored_target("6.000000000000001", std::nextafter(6., 7.));
        every_api_requires_config(nullptr);
        {
            const io::Value empty{io::Value::Object{}};
            every_api_requires_config(&empty);
        }
        {
            const io::Value schema_only{document(
                    {{"schema", integer(1)}})};
            every_api_requires_config(&schema_only);
        }
        for (const char *key : {"target_speed_kmh", "kp_nm_per_mps",
                                "ki_nm_per_mps_s", "torque_ceiling_nm"})
            missing_config_key_names_it(key);
        schema_and_section_validation(io::Value{document(
                {{"cruise", cruise_section()}})});
        schema_and_section_validation(io::Value{document(
                {{"schema", integer(3)}, {"cruise", cruise_section()}})});
        schema_and_section_validation(io::Value{document(
                {{"schema", integer(1)},
                 {"cruise", io::Value{io::Value::Array{}}}})});
        for (const char *key : {"target_speed_kmh", "kp_nm_per_mps",
                                "ki_nm_per_mps_s", "torque_ceiling_nm"})
            for (const double value : {0., -1., kNan, kInf, -kInf})
                invalid_config_value(key, value);
        config_target_band(14.999);
        config_target_band(45.001);
        for (const auto &[name, kind] : std::array{
                     std::pair{"other", "slide"}, std::pair{"root_x", "ball"},
                     std::pair{"root_x", "free"}})
            root_joint_validation(name, kind);
        root_joint_validation("root_x", "hinge");
        for (const char *method :
             {"cruise_set_target_speed", "cruise_set_assist_compensation"})
            for (const double value : {kNan, kInf, -kInf})
                nonfinite_setters_preserve_state(method, value);
        target_setter_band_preserves_state(14.999);
        target_setter_band_preserves_state(45.001);
        for (const char *key : {"target_speed_mps", "integral_mps_s",
                                "torque_nm", "gain_scale"})
            for (const double value : {kNan, kInf, -kInf})
                nonfinite_restore_preserves_state(key, value);
        invalid_restore_preserves_state();
        test_support_underflow_preserves_state();
        test_projection_optional_cruise();
    });
}
