// test_native_tire.cpp — C++ port of tests/reference/test_native_tire.py
// (batch 3.13).
//
// Native compliant_2d tyre writer parity: the golden episode replay
// (qfrc + evolved wire + snapshots + resistance components bitwise
// against the captured artifact — the artifact's own rows ARE the
// oracle; the Python file's live-applier half is a capture-time check
// that proved artifact == oracle) plus the config/state/clock contract
// surface on the checked-in bench model.
#include "tyre_reference_support.hpp"

#include "io/json.hpp"
#include "writers/resistance.hpp"
#include "writers/tire_common.hpp"

#include <cmath>

namespace {

using namespace tyre_reference;
namespace io = bike::io;

// The flat names the writer emits/consumes — flatten_row(asdict(_BrushState))
// per side: xi, tangent, point, segment, center (CANONICAL_NAMES).
[[nodiscard]] std::vector<std::string> canonical_names() {
    std::vector<std::string> out;
    for (const char *side : {"front", "rear"})
        for (const char *field :
             {"xi", "tangent.0", "tangent.1", "tangent.2", "point.0",
              "point.1", "point.2", "segment", "center.0", "center.1",
              "center.2"})
            out.emplace_back(std::string(side) + "." + field);
    return out;
}

[[nodiscard]] std::vector<double>
tire_row(const Episode &ep, std::size_t k) {
    return {row(ep.tire_state, k, ep.wire).begin(),
            row(ep.tire_state, k, ep.wire).end()};
}

// _tire_cfg(*missing) — the minimal compliant_2d constant-mu section on
// the pinned bench model.
[[nodiscard]] std::string tire_cfg_json(std::string_view drop = {}, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                                        std::string_view front_drop = {},
                                        std::string_view surface_mode =
                                                "configured") {
    const std::string material =
            R"({"radial_k_n_m":130000.0,"radial_c_ns_m":800.0,)"
            R"("pressure_pa_gauge":200000.0,"provenance":"synthetic",)"
            R"("valid_load_range_n":[0.0,2000.0]},)";
    const auto side = [&](std::string_view omit) {
        std::string s = "{";
        s += omit == "material" ? "" : std::string("\"material\":") +
                                                material;
        s += omit == "tangent_k_n_m" ? "" : R"("tangent_k_n_m":20000.0,)";
        s += omit == "mu" ? "" : R"("mu":0.8,)";
        s += omit == "relaxation_length_m"
                     ? ""
                     : R"("relaxation_length_m":0.2,)";
        if (!s.empty() && s.back() == ',') s.pop_back();
        s += '}';
        return s;
    };
    std::string tire = "{";
    if (drop != "backend") tire += R"("backend":"compliant_2d",)";
    if (drop != "surface_mode")
      tire += R"("surface_mode":")" + std::string(surface_mode) + "\",";
    if (drop != "significant_delta_m")
        tire += R"("significant_delta_m":0.0001,)";
    if (drop != "significance_fraction")
        tire += R"("significance_fraction":0.05,)";
    if (drop != "distinct_normal_deg")
        tire += R"("distinct_normal_deg":20.0,)";
    if (drop != "front") tire += "\"front\":" + side(front_drop) + ",";
    if (drop != "rear") tire += "\"rear\":" + side(front_drop) + ",";
    if (tire.back() == ',') tire.pop_back();
    tire += '}';
    return R"({"schema":1,"tire":)" + tire + "}";
}

[[nodiscard]] nativecfg::WriterConfig cfg(std::string_view drop = {},
                                          std::string_view front_drop = {},
                                          std::string_view surface_mode =
                                                  "configured") {
    return nativecfg::decode(
            io::parse_json(tire_cfg_json(drop, front_drop, surface_mode)));
}

[[nodiscard]] const std::string &mjb() {
    static const std::string path =
            repo_file("tests/fixtures/benchmark/model.mjb")
                    .string();
    return path;
}

// Slide root_x/rider_root_x +0.6 so both wheels sit over the road, then
// set_state + forward — the happy-path preamble the contract tests share.
void slide_and_forward(Stepper &st) {
    std::vector<double> qpos(static_cast<std::size_t>(st.model()->nq), 0.);
    qpos[0] += 0.6;
    qpos[14] += 0.6;
    std::vector<double> qvel(static_cast<std::size_t>(st.model()->nv), 0.);
    std::vector<double> act(static_cast<std::size_t>(st.model()->na), 0.);
    std::vector<double> warmstart(
            static_cast<std::size_t>(st.model()->nv), 0.);
    st.set_state(qpos, qvel, act, warmstart, 0.);
    st.forward();
}

[[nodiscard]] std::size_t index_of(
        const std::vector<std::string> &names, std::string_view leaf) {
    return channel(std::span<const std::string>(names), leaf);
}

// ---------------------------------------------------------------------
// tests/reference/test_native_tire.py::test_tire_qfrc_bitwise — the
// golden episode replay. Artifact rows are the oracle goldens.
// ---------------------------------------------------------------------
[[gnu::noinline]] void test_tire_qfrc_bitwise() {
    auto golden = node("test_native_tire.py", "test_tire_qfrc_bitwise");
    const auto ep = load_episode_bundle(
            platform_fixture("native_tire", "episodes/stand40"));
    const auto config = load_config(
            platform_fixture("native_tire", "env"));
    Stepper st((platform_fixture("native_tire", "episodes/stand40") /
                "model.mjb")
                       .string(),
               config);
    const auto &names = ep.artifact.tire_state_names;
    const double dt = /* ref.opt.timestep */ .00125;
    const std::size_t steps = ep.artifact.state_qpos.shape[0];
    for (std::size_t k = 0; k < steps; ++k) {
        const std::string tag = std::to_string(k);
        st.set_state(state_row(ep.artifact.state_qpos, k, ep.nq),
                     state_row(ep.artifact.state_qvel, k, ep.nv),
                     state_row(ep.artifact.state_act, k, ep.na),
                     state_row(ep.artifact.state_warmstart, k, ep.nv),
                     ep.time[k]);
        st.forward();
        const auto consumed = tire_row(ep, k);
        st.set_tire_state(names, consumed);
        const auto got = st.tire_qfrc(dt);
        vector(golden, got, "tires golden step " + tag);
        // The live-oracle leaf carries the same bits; the port consumes
        // it against the same native vector.
        vector(golden, got, "tires live-oracle step " + tag);
        const auto evolved = st.tire_state();
        vector(golden, evolved, "tire_state evolve step " + tag);
        if (k + 1 < steps)
            vector(golden, evolved, "tire_state[k+1] step " + tag);
        // Contact snapshots equal the manifest digest bitwise (the
        // recorded leaf stream), then feed the resistance writer with
        // the same schema the binding used.
        snapshots(golden, st, tag);
        const auto snaps = st.tire_writer().snapshots();
        std::array<std::vector<double>, 2> loads;
        std::array<std::unique_ptr<bool[]>, 2> working; // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        std::array<TireSideInput, 2> side_input;
        for (int side = 0; side < 2; ++side) {
            const auto &s = snaps.at(static_cast<std::size_t>(side)).value(); // NOLINT(bugprone-unchecked-optional-access) snap engagement is the fixture contract under test; .value() reports violations loudly
            auto &lds = loads.at(static_cast<std::size_t>(side));
            auto const &wrk = working.at(static_cast<std::size_t>(side)) =
                std::make_unique<bool[]>(s.patches.size()); // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
            const std::span<bool> wrk_view = std::views::counted(
                    wrk.get(), static_cast<std::ptrdiff_t>(
                                       s.patches.size()));
            for (std::size_t i = 0; i < s.patches.size(); ++i) {
                lds.push_back(s.patches[i].normal_load_n);
                wrk_view[i] = s.patches[i].working_surface;
            }
            side_input.at(static_cast<std::size_t>(side)) = TireSideInput{
                    .patch_loads = lds,
                    .patch_working = wrk_view,
                    .effective_radius_m = s.effective_radius_m};
        }
        for (const auto &[name, values] :
             st.resistance_components(side_input[0], side_input[1]))
            vector(golden, values,
                   name + " via tire_snapshots step " + tag); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
    }
    golden.done();
}

// ---------------------------------------------------------------------
// tests/reference/test_native_tire.py::test_tire_state_roundtrip_and_clock
// ---------------------------------------------------------------------
[[gnu::noinline]] void test_tire_state_roundtrip_and_clock() {
    auto golden = node("test_native_tire.py",
                       "test_tire_state_roundtrip_and_clock");
    const auto ep = load_episode_bundle(
            platform_fixture("native_tire", "episodes/stand40"));
    const auto config = load_config(
            platform_fixture("native_tire", "env"));
    Stepper st((platform_fixture("native_tire", "episodes/stand40") /
                "model.mjb")
                       .string(),
               config);
    const auto &names = ep.artifact.tire_state_names;
    const double dt = .00125;
    bt::require(st.tire_state_names() == canonical_names(),
                "writer emits the canonical 22-name wire");
    const std::size_t steps = ep.artifact.state_qpos.shape[0];
    for (std::size_t k = 0; k < steps; ++k) {
        st.set_state(state_row(ep.artifact.state_qpos, k, ep.nq),
                     state_row(ep.artifact.state_qvel, k, ep.nv),
                     state_row(ep.artifact.state_act, k, ep.na),
                     state_row(ep.artifact.state_warmstart, k, ep.nv),
                     ep.time[k]);
        st.forward();
        st.set_tire_state(names, tire_row(ep, k));
        vector(golden, st.tire_state(),
               "state roundtrip step " + std::to_string(k));
    }
    golden.done();
    // last_time_s discipline: advancing twice at one timestamp is an
    // error; a fresh set_tire_state restores the clock.
    st.set_state(state_row(ep.artifact.state_qpos, 0, ep.nq),
                 state_row(ep.artifact.state_qvel, 0, ep.nv),
                 state_row(ep.artifact.state_act, 0, ep.na),
                 state_row(ep.artifact.state_warmstart, 0, ep.nv),
                 ep.time[0]);
    st.forward();
    st.set_tire_state(names, tire_row(ep, 0));
    (void)st.tire_qfrc(dt);
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                   "once per increasing timestamp");
    st.set_tire_state(names, tire_row(ep, 0));
    (void)st.tire_qfrc(dt);
}

// ---------------------------------------------------------------------
// Config + state contracts on the pinned bench model.
// ---------------------------------------------------------------------

// tests/reference/test_native_tire.py::test_tire_requires_config
[[gnu::noinline]] void test_tire_requires_config() {
    std::vector<double> row(22, 0.);
    const auto names = canonical_names();
    for (auto const make : {0, 1}) {
      std::optional<Stepper> st;
      if (make == 0)
        st.emplace(mjb());
      else
        st.emplace(mjb(), nativecfg::WriterConfig{});
      rejects<std::logic_error>([&] { (void)st->tire_qfrc(.00125); });
      rejects<std::logic_error>([&] { st->set_tire_state(names, row); });
      rejects<std::logic_error>([&] { (void)st->tire_state(); });
      rejects<std::logic_error>([&] { (void)st->tire_writer().snapshots(); });
    }
}

// tests/reference/test_native_tire.py::test_tire_config_missing_key_names_it
// Stepper construction holds the writer decode + engine image — over the
// 8192-byte frame budget this file otherwise keeps.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[gnu::noinline]] void test_tire_config_missing_key_names_it() {
    rejects<std::invalid_argument>(
            [] { (void)Stepper(mjb(), cfg("backend")); }, "backend");
    rejects<std::invalid_argument>(
            [] { (void)Stepper(mjb(), cfg("surface_mode")); },
            "surface_mode");
    rejects<std::invalid_argument>(
            [] { (void)Stepper(mjb(), cfg("", "mu")); }, "mu");
    // track mode without a serialized surface_map is rejected like the
    // Python applier's 'track material mode requires an explicit
    // SurfaceMap'.
    rejects<std::invalid_argument>(
            [] { (void)Stepper(mjb(), cfg("", "", "track")); });
}
NATIVE_DIAG_POP

// tests/reference/test_native_tire.py::test_tire_set_state_validation
[[gnu::noinline]] void test_tire_set_state_validation() {
    Stepper st(mjb(), cfg());
    const auto names = canonical_names();
    std::vector<double> row(22, 0.);
    st.set_tire_state(names, row);
    rejects<std::invalid_argument>([&] {
        st.set_tire_state(
                std::span<const std::string>(names).first(names.size() - 1),
                row);
    });
    rejects<std::invalid_argument>([&] {
        st.set_tire_state(names, std::span<const double>(row).first(21));
    });
    auto bogus = names;
    bogus[0] = "bogus.xi";
    rejects<std::invalid_argument>(
            [&] { st.set_tire_state(bogus, row); }, "front.xi");
}

// tests/reference/test_native_tire.py::test_tire_segment_rejects_invalid_without_mutation[14]
[[gnu::noinline]] void test_tire_segment_rejects_invalid_without_mutation() {
    const std::array<std::pair<double, std::string_view>, 7> segments{{
            {std::numeric_limits<double>::infinity(), "inf"},
            {-std::numeric_limits<double>::infinity(), "-inf"},
            {0.5, "0.5"},
            {-0.5, "-0.5"},
            {2147483648., "2147483648.0"},
            {-2147483649., "-2147483649.0"},
            {std::numeric_limits<double>::max(),
             "1.7976931348623157e+308"}}};
    const auto names = canonical_names();
    for (const char *side : {"front", "rear"})
        for (const auto &[segment, seg_id] : segments) {
            auto golden = node(
                    "test_native_tire.py",
                    "test_tire_segment_rejects_invalid_without_mutation",
                    std::string(seg_id) + "-" + side);
            Stepper st(mjb(), cfg());
            slide_and_forward(st);
            std::vector<double> row(22, 0.);
            st.set_tire_state(names, row);
            (void)st.tire_qfrc(.00125);
            const auto before = st.tire_state();
            auto bad = before;
            bad[0] += 0.123; // rejected late fields must not commit earlier
            bad[index_of(names, std::string(side) + ".segment")] = segment;
            rejects<std::invalid_argument>(
                    [&] { st.set_tire_state(names, bad); },
                    std::string(side) + ".segment");
            vector(golden, st.tire_state(), "rejected segment state");
            golden.done();
            rejects<std::invalid_argument>(
                    [&] { (void)st.tire_qfrc(.00125); },
                    "once per increasing timestamp");
        }
}

// tests/reference/test_native_tire.py::test_tire_segment_int_boundaries_and_unset_roundtrip[10]
[[gnu::noinline]] void test_tire_segment_int_boundaries_and_unset_roundtrip() {
    const std::array<std::pair<double, std::string_view>, 5> segments{{
            {-2147483648., "-2147483648.0"},
            {-1., "-1.0"},
            {0., "0.0"},
            {2147483647., "2147483647.0"},
            {std::numeric_limits<double>::quiet_NaN(), "nan"}}};
    const auto names = canonical_names();
    for (const char *side : {"front", "rear"})
        for (const auto &[segment, seg_id] : segments) {
            auto golden = node(
                    "test_native_tire.py",
                    "test_tire_segment_int_boundaries_and_unset_roundtrip",
                    std::string(seg_id) + "-" + side);
            Stepper st(mjb(), cfg());
            std::vector<double> row(22, 0.);
            row[index_of(names, std::string(side) + ".segment")] = segment;
            if (std::isnan(segment))
                // Unset segment is only emitted inside a fully-unset
                // contact group — a stored tangent without its segment
                // is a rejected partial group.
                for (std::size_t i = 0; i < names.size(); ++i)
                    if (names[i].starts_with(std::string(side) + ".") &&
                        !names[i].ends_with(".xi"))
                        row[i] = std::numeric_limits<double>::quiet_NaN();
            st.set_tire_state(names, row);
            vector(golden, st.tire_state(), "segment roundtrip");
            golden.done();
        }
}

// tests/reference/test_native_tire.py::test_tire_qfrc_happy_path_and_clock
[[gnu::noinline]] void test_tire_qfrc_happy_path_and_clock() {
    Stepper st(mjb(), cfg());
    slide_and_forward(st);
    const auto names = canonical_names();
    st.set_tire_state(names, std::vector<double>(22, 0.));
    const double dt = .00125;
    const auto q = st.tire_qfrc(dt);
    bt::require(q.size() == static_cast<std::size_t>(st.model()->nv),
                "qfrc is nv wide");
    rejects<std::invalid_argument>(
            [&] { (void)st.tire_qfrc(dt); });
    st.forward();
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(0.); },
                                   "tire interval");
    // The binding spells this ValueError 'tire_qfrc.dt'
    // (wire::finite_real); the writer's scalar(dt,'tire interval')
    // validator is what fires inside the Stepper — same rejection.
    rejects<std::invalid_argument>([&] {
        (void)st.tire_qfrc(std::numeric_limits<double>::quiet_NaN());
    }, "tire interval");
    // Time advanced: the interval check passes and compute runs again.
    std::vector<double> qpos(static_cast<std::size_t>(st.model()->nq), 0.);
    qpos[0] += 0.6;
    qpos[14] += 0.6;
    std::vector<double> qvel(static_cast<std::size_t>(st.model()->nv), 0.);
    std::vector<double> act(static_cast<std::size_t>(st.model()->na), 0.);
    std::vector<double> warm(static_cast<std::size_t>(st.model()->nv), 0.);
    st.set_state(qpos, qvel, act, warm, 0. + dt);
    st.forward();
    (void)st.tire_qfrc(dt);
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                   "once per increasing timestamp");
}

// tests/reference/test_native_tire.py::test_tire_partial_sentinel_group_rejects_atomically[4]
[[gnu::noinline]] void test_tire_partial_sentinel_group_rejects_atomically() {
    const auto names = canonical_names();
    for (const char *side : {"front", "rear"})
        for (const char *blanked : {"point", "segment"}) {
            auto golden = node(
                    "test_native_tire.py",
                    "test_tire_partial_sentinel_group_rejects_atomically",
                    std::string(blanked) + "-" + side);
            Stepper st(mjb(), cfg());
            slide_and_forward(st);
            st.set_tire_state(names, std::vector<double>(22, 0.));
            (void)st.tire_qfrc(.00125);
            const auto before = st.tire_state();
            std::vector<double> bad(22, 0.);
            const std::string prefix = std::string(side) + ".";
            if (std::string_view(blanked) == "point")
                for (int i = 0; i < 3; ++i)
                    bad[index_of(names, prefix + "point." +
                                           std::to_string(i))] =
                            std::numeric_limits<double>::quiet_NaN();
            else
                bad[index_of(names, prefix + "segment")] =
                        std::numeric_limits<double>::quiet_NaN();
            rejects<std::invalid_argument>(
                    [&] { st.set_tire_state(names, bad); },
                    prefix + "tangent");
            vector(golden, st.tire_state(), "rejected sentinel group");
            golden.done();
            rejects<std::invalid_argument>(
                    [&] { (void)st.tire_qfrc(.00125); },
                    "once per increasing timestamp");
        }
}

// tests/reference/test_native_tire.py::test_tire_xi_and_mixed_vector_rejections_preserve_state
[[gnu::noinline]] void test_tire_xi_and_mixed_vector_rejections_preserve_state() {
    auto golden = node(
            "test_native_tire.py",
            "test_tire_xi_and_mixed_vector_rejections_preserve_state");
    Stepper st(mjb(), cfg());
    const auto names = canonical_names();
    st.set_tire_state(names, std::vector<double>(22, 0.));
    const auto before = st.tire_state();
    std::vector<double> bad(22, 0.);
    bad[index_of(names, "front.xi")] =
            std::numeric_limits<double>::quiet_NaN();
    rejects<std::invalid_argument>([&] { st.set_tire_state(names, bad); },
                                   "front.xi");
    // A real vector column can carry a stray NaN and still decodes — but
    // a mixed finite/NaN vector is neither set nor unset.
    bad = std::vector<double>(22, 0.);
    bad[index_of(names, "rear.point.1")] =
            std::numeric_limits<double>::quiet_NaN();
    rejects<std::invalid_argument>([&] { st.set_tire_state(names, bad); },
                                   "finite or all-NaN");
    vector(golden, st.tire_state(), "rejected rows preserve state");
    golden.done();
}

// tests/reference/test_native_tire.py::test_tire_all_unset_state_accepted_and_first_advance_works
[[gnu::noinline]] void test_tire_all_unset_state_accepted_and_first_advance_works() {
    Stepper st(mjb(), cfg());
    slide_and_forward(st);
    const auto names = canonical_names();
    std::vector<double> row(22, std::numeric_limits<double>::quiet_NaN());
    row[index_of(names, "front.xi")] = 0.;
    row[index_of(names, "rear.xi")] = 0.;
    st.set_tire_state(names, row);
    const auto got = st.tire_state();
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (names[i].ends_with(".xi"))
            bt::require(got[i] == 0., names[i]);
        else
            bt::require(std::isnan(got[i]), names[i]);
    }
    bt::require(st.tire_qfrc(.00125).size() ==
                    static_cast<std::size_t>(st.model()->nv),
                "first advance works");
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(.00125); },
                                   "once per increasing timestamp");
}

} // namespace

int main() {
    return bt::run([] {
        test_tire_qfrc_bitwise();
        test_tire_state_roundtrip_and_clock();
        test_tire_requires_config();
        test_tire_config_missing_key_names_it();
        test_tire_set_state_validation();
        test_tire_segment_rejects_invalid_without_mutation();
        test_tire_segment_int_boundaries_and_unset_roundtrip();
        test_tire_qfrc_happy_path_and_clock();
        test_tire_partial_sentinel_group_rejects_atomically();
        test_tire_xi_and_mixed_vector_rejections_preserve_state();
        test_tire_all_unset_state_accepted_and_first_advance_works();
    });
}
