// test_native_pneumatic_tire.cpp — C++ port of
// tests/reference/test_native_pneumatic_tire.py (batch 3.12b).
//
// Replays the pneumatic tire writer end to end: config projection and
// rejection, the state wire schema/round-trip/atomicity, per-state golden
// episode parity (qfrc, evolved wire, snapshots, diagnostics, energy
// counters), the detailed-tier two-wheel rig sweeps, the probe/clock
// contract and the multi-contact observations publication.
//
// Inputs ride the golden stream (label prefix "input.") or the
// platform-keyed artifact bundle under
// tests/fixtures/golden/pneumatic_tire/ captured by the
// additive _fixture_save_* hooks in the Python file.
#include "diag.hpp"
#include "pneumatic_reference_support.hpp"

#include <map>
#include <set>

#include "engine_call.hpp"           // engine::load_model_image
#include "golden/episode.hpp"
#include "io/json.hpp"
#include "io/npy.hpp"
#include "platform/files.hpp"
#include "research/environment.hpp"
#include "runtime/control.hpp"       // runtime::WireObject
#include "runtime/research_io.hpp"
#include "runtime/runtime.hpp"
#include "stepper.hpp"
#include "writer_config.hpp"         // nativecfg::decode

namespace {

using namespace pneumatic_reference;
constexpr std::string_view kFile = "test_native_pneumatic_tire.py";
namespace io = bike::io;
namespace golden = bike::golden;

// ---------------------------------------------------------------------
// Fixture bundle / golden entry plumbing
// ---------------------------------------------------------------------

// The shared artifact bundles the Python capture writes: env_fast
// carries wire.json/physics.toml/request.toml (the settled
// initial_state.json is per-platform); episodes/<name> holds the replay
// episode. The episode rows are the live platform pipeline's output —
// numba/LLVM codegen differs between darwin-arm64 and linux-aarch64 —
// so episodes live under <platform>/pneumatic_tire/, not the shared
// tree. rig_detailed/rig_events carry the small-rig model and wire.
[[nodiscard]] std::filesystem::path fixture(std::string_view name) {
    return bt::oracle::root() / "pneumatic_tire" / std::string(name);
}

// Per-platform bundle: tests/fixtures/golden/<platform>/pneumatic_tire/.
[[nodiscard]] std::filesystem::path platform_fixture(std::string_view name) {
    return bt::oracle::root() / bt::oracle::platform() / "pneumatic_tire" /
            std::string(name);
}

[[nodiscard]] std::vector<std::byte> fixture_bytes(
        const std::filesystem::path &path) {
    auto bytes = io::read_numpy_bytes(path);
    bt::require(!bytes.empty(),
                "fixture artifact missing or empty: " + path.string());
    return bytes;
}

[[nodiscard]] std::string fixture_text(const std::filesystem::path &path) {
    const auto bytes = fixture_bytes(path);
    return {reinterpret_cast<const char *>(bytes.data()), bytes.size()};
}

// rig_* configs are deterministic text and stay shared.
[[nodiscard]] nativecfg::WriterConfig load_config(std::string_view env) {
    return nativecfg::decode(io::parse_json(
            fixture_text(fixture(env) / "wire.json")));
}

// env_* wires are projected by the platform pipeline — per-platform.
[[nodiscard]] nativecfg::WriterConfig platform_config(std::string_view env) {
    return nativecfg::decode(io::parse_json(
            fixture_text(platform_fixture(env) / "wire.json")));
}

[[nodiscard]] std::string join_lines(std::span<const std::string> names) {
    std::string out;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i) out.push_back('\n');
        out += names[i];
    }
    return out;
}

// _record_string(value, label) — a |u1 bytes entry.
inline void text(bt::oracle::Node &golden, std::string_view actual,
                 const std::string &label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(std::as_bytes(std::span(actual)), "|u1", shape, label);
}

// _record_int(value, label) — a 0-d <i8 entry.
inline void count(bt::oracle::Node &golden, std::int64_t actual,
                  const std::string &label) {
    const auto bits = std::bit_cast<std::uint64_t>(actual);
    std::array<std::byte, 8> bytes;
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes[shift / 8] = static_cast<std::byte>((bits >> shift) & 0xffu);
    golden.bytes(bytes, "<i8", {}, label);
}

// ---------------------------------------------------------------------
// _assert_snapshots_equal / _assert_diagnostics_equal — the record order
// is the oracle dicts' insertion order; the C++ emits each comparison in
// that same order.
// ---------------------------------------------------------------------

// _assert_snapshots_equal(native, row, k): per side 'front' then 'rear';
// interval_id, backend, geometric_contact, time_s, effective_radius_m,
// patches count, then per-patch working_surface + the three floats.
// `tag` is "step {k}" or "tick {k}" / a sweep label.
void snapshots(bt::oracle::Node &golden, const Stepper &st,
               const std::string &tag) {
    const auto snaps = st.tire_writer().snapshots();
    const std::array<std::string_view, 2> sides{"front", "rear"};
    for (int side = 0; side < 2; ++side) {
        const auto &snap = snaps.at(static_cast<std::size_t>(side));
        bt::require(snap.has_value(),
                    std::string(sides.at(static_cast<std::size_t>(side))) +
                            " snapshot present");
        const auto &s = *snap;
        const std::string base = std::string(
                sides.at(static_cast<std::size_t>(side)));
        count(golden, s.interval_id, base + ".interval_id " + tag);
        text(golden, s.backend, base + ".backend " + tag);
        boolean(golden, s.geometric_contact,
                base + ".geometric_contact " + tag);
        scalar(golden, s.time_s, base + ".time_s " + tag);
        scalar(golden, s.effective_radius_m,
               base + ".effective_radius_m " + tag);
        count(golden, static_cast<std::int64_t>(s.patches.size()),
              base + ".patches " + tag);
        for (std::size_t i = 0; i < s.patches.size(); ++i) {
            const auto &p = s.patches[i];
            const std::string pbase =
                    base + ".patches[" + std::to_string(i) + "]";
            boolean(golden, p.working_surface,
                    pbase + ".working_surface " + tag);
            scalar(golden, p.normal_load_n,
                   pbase + ".normal_load_n " + tag);
            scalar(golden, p.tangent_force_n,
                   pbase + ".tangent_force_n " + tag);
            scalar(golden, p.slip_mps, pbase + ".slip_mps " + tag);
        }
    }
}

// _assert_diagnostics_equal — the pneumatic schema's 28-key insertion
// order (pneumatic_tire_forces.py:426-455); per-key capture kind is
// fixed by the oracle's Python value types: floats -> <f8, patch_count
// -> <i8, the flags -> |b1, surface -> |u1.
void diagnostics(bt::oracle::Node &golden, const Stepper &st,
                 const std::string &tag) {
    const auto diags = st.tire_writer().diagnostics();
    const std::array<std::string_view, 2> sides{"front", "rear"};
    for (int side = 0; side < 2; ++side) {
        const auto &slot = diags.at(static_cast<std::size_t>(side));
        bt::require(slot.has_value(),
                    std::string(sides.at(static_cast<std::size_t>(side))) +
                            " diagnostics present");
        const auto &d = *slot;
        const auto at = [&](std::string_view key) {
            return std::string(
                           sides.at(static_cast<std::size_t>(side))) +
                   "." + std::string(key) + " " + tag;
        };
        scalar(golden, d.normal_load_n, at("normal_load_n"));
        scalar(golden, d.support_n, at("support_n"));
        scalar(golden, d.tangent_force_n, at("tangent_force_n"));
        scalar(golden, d.slip_mps, at("slip_mps"));
        scalar(golden, d.slip_ratio, at("slip_ratio"));
        scalar(golden, d.transient_slip_ratio,
               at("transient_slip_ratio"));
        scalar(golden, d.friction_coefficient,
               at("friction_coefficient"));
        scalar(golden, d.penetration_m, at("penetration_m"));
        scalar(golden, d.contact_length_m, at("contact_length_m"));
        count(golden, d.patch_count, at("patch_count"));
        boolean(golden, d.multi_support, at("multi_support"));
        boolean(golden, d.supports_multiple_contacts,
                at("supports_multiple_contacts"));
        boolean(golden, d.fully_sliding, at("fully_sliding"));
        boolean(golden, d.airborne, at("airborne"));
        boolean(golden, d.coverage_event, at("coverage_event"));
        boolean(golden, d.rim_strike_active, at("rim_strike_active"));
        scalar(golden, d.rim_event_x_m, at("rim_event_x_m"));
        scalar(golden, d.rim_event_speed_mps, at("rim_event_speed_mps"));
        scalar(golden, d.rim_event_peak_load_n,
               at("rim_event_peak_load_n"));
        scalar(golden, d.rim_event_peak_force_n,
               at("rim_event_peak_force_n"));
        scalar(golden, d.rim_event_energy_j, at("rim_event_energy_j"));
        scalar(golden, d.pressure_bar, at("pressure_bar"));
        scalar(golden, d.dissipated_power_w, at("dissipated_power_w"));
        scalar(golden, d.brush_loss_j, at("brush_loss_j"));
        scalar(golden, d.radial_energy_j, at("radial_energy_j"));
        scalar(golden, d.shear_energy_j, at("shear_energy_j"));
        boolean(golden, d.outside_material_load_range,
                at("outside_material_load_range"));
        text(golden, d.surface, at("surface"));
    }
}

// ---------------------------------------------------------------------
// Golden-episode bundle
// ---------------------------------------------------------------------

struct Episode {
    golden::EpisodeArtifact artifact;
    std::vector<double> qpos, qvel, act, warmstart, time;
    std::vector<double> tire_state, forces;
    std::size_t nq = 0, nv = 0, na = 0, wire = 0, nfor = 0, fwidth = 0;
};

[[nodiscard]] Episode load_episode_bundle(std::string_view name) {
    Episode ep;
    ep.artifact = golden::load_episode(
            platform_fixture("episodes") / std::string(name));
    // Golden episodes can carry nonfinite cells (a NaN-for-none control
    // channel or warmstart); the codec reads them with allow_nonfinite,
    // and the parity rows here need the same.
    ep.qpos = ep.artifact.state_qpos.doubles(true);
    ep.qvel = ep.artifact.state_qvel.doubles(true);
    ep.act = ep.artifact.state_act.doubles(true);
    ep.warmstart = ep.artifact.state_warmstart.doubles(true);
    ep.time = ep.artifact.state_time.doubles(true);
    ep.tire_state = ep.artifact.tire_state.doubles(true);
    ep.forces = ep.artifact.forces.doubles(true);
    ep.nq = ep.artifact.state_qpos.shape.at(1);
    ep.nv = ep.artifact.state_qvel.shape.at(1);
    ep.na = ep.artifact.state_act.shape.size() == 2
                    ? ep.artifact.state_act.shape[1]
                    : 0;
    ep.wire = ep.artifact.tire_state.shape.at(1);
    ep.nfor = ep.artifact.forces.shape.at(1);
    ep.fwidth = ep.artifact.forces.shape.size() == 3
                        ? ep.artifact.forces.shape[2]
                        : 0;
    bt::require(ep.fwidth > 0, "episode forces is (steps,channels,nv)");
    return ep;
}

[[nodiscard]] std::span<const double>
row(const std::vector<double> &table, std::size_t k, std::size_t width) {
    return std::span<const double>(table).subspan(k * width, width);
}

[[nodiscard]] std::size_t channel(const std::vector<std::string> &names,
                                  std::string_view leaf) {
    const auto it = std::ranges::find(names, leaf);
    bt::require(it != names.end(),
                "tire_state_names carries " + std::string(leaf));
    return static_cast<std::size_t>(it - names.begin());
}

// The consumed patch (start, stop) bound tuples of one wire row —
// _patch_bounds in the oracle file.
[[nodiscard]] std::vector<std::pair<int, int>>
patch_bounds(std::span<const double> wire_row,
             const std::vector<std::string> &names, std::string_view side) {
    const auto count = static_cast<int>(
            wire_row[channel(names, std::string(side) + ".patch.count")]);
    std::vector<std::pair<int, int>> out;
    for (int j = 0; j < count; ++j) {
        const auto tag = std::string(side) + ".patch." +
                (j < 10 ? "0" : "") + std::to_string(j);
        out.emplace_back(
                static_cast<int>(wire_row[channel(names, tag + ".start")]),
                static_cast<int>(wire_row[channel(names, tag + ".stop")]));
    }
    return out;
}

// _run_golden_parity: names/force_names records, then per step k: qfrc
// vs the golden row and the live-oracle row (two identical entries),
// the evolved wire vs the committed applier dict and the next recorded
// row, snapshots, diagnostics twice (manifest digest + live dict — the
// same stream), then the three energy surfaces.
void golden_parity(bt::oracle::Node &golden, std::string_view episode_name,
                   std::string_view env_name, std::size_t rows = 0) {
    auto ep = load_episode_bundle(episode_name);
    // The env model was a byte-identical copy of the episode model; the
    // deduplicated bundle keeps only the episode copy.
    const auto mjb = fixture_bytes(platform_fixture("episodes") /
                                   std::string(episode_name) / "model.mjb");
    const auto cfg = platform_config(env_name);

    Stepper st(std::span<const std::byte>(mjb), cfg);
    const std::size_t steps = ep.artifact.state_qpos.shape.at(0);
    bt::require(st.tire_state_names() == ep.artifact.tire_state_names,
                "stepper names match the artifact's");
    text(golden, join_lines(ep.artifact.tire_state_names), "names");
    text(golden, join_lines(ep.artifact.force_names), "force_names");
    const double dt = st.model()->opt.timestep;
    if (rows == 0 || rows > steps) rows = steps;

    for (std::size_t k = 0; k < rows; ++k) {
        const std::string tag = "step " + std::to_string(k);
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na),
                     row(ep.warmstart, k, ep.nv), ep.time[k]);
        st.forward();
        st.set_tire_state(ep.artifact.tire_state_names,
                          row(ep.tire_state, k, ep.wire));
        const auto got = st.tire_qfrc(dt);
        vector(golden, got, "tires golden step " + std::to_string(k));
        vector(golden, got,
               "tires live-oracle step " + std::to_string(k));
        const auto wire = st.tire_state();
        vector(golden, wire,
               "tire_state evolve step " + std::to_string(k));
        if (k + 1 < steps)
            vector(golden, wire,
                   "tire_state golden evolve step " + std::to_string(k));
        snapshots(golden, st, tag);
        diagnostics(golden, st, tag);   // the manifest digest
        diagnostics(golden, st, tag);   // the committed live dict
        scalar(golden, st.tire_stored_energy(),
               "stored_energy step " + std::to_string(k));
        scalar(golden, st.tire_brush_loss_step_j(),
               "brush_loss_step_j step " + std::to_string(k));
        scalar(golden, st.tire_radial_dissipation_power_w(),
               "radial_dissipation_power_w step " + std::to_string(k));
    }
}

// ---------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------

// test_config_projection_carries_the_pneumatic_section
void test_config_projection_carries_the_pneumatic_section() {
    auto golden = node(
            kFile, "test_config_projection_carries_the_pneumatic_section");
    const auto wire =
            io::parse_json(fixture_text(platform_fixture("env_fast") / "wire.json"));
    const auto &tire = io::as_object(
            *io::find_member(io::as_object(wire, "wire"), "tire"), "tire");
    const auto *backend = io::find_member(tire, "backend");
    bt::require(backend != nullptr &&
                        io::as_string(*backend, "tire.backend") ==
                                "pneumatic",
                "backend is pneumatic");
    const auto &front =
            io::as_object(*io::find_member(tire, "front"), "tire.front");
    bt::require(front.size() == 3 && io::find_member(front, "mu") &&
                        io::find_member(front, "tangent_k_n_m") &&
                        io::find_member(front, "relaxation_length_m"),
                "front carries exactly the scalar tire keys");
    const auto &pneumatic = io::as_object(
            *io::find_member(tire, "pneumatic"), "tire.pneumatic");
    bt::require(io::find_member(pneumatic, "surface") == nullptr,
                "no resolved surface name on the wire");
    const auto cfg = nativecfg::decode(wire);
    bt::require(cfg.tire.has_value() && cfg.tire->pneumatic.has_value(),
                "pneumatic section decoded");
    const auto &p = *cfg.tire->pneumatic;
    // The resolved tier row as the golden gate — [n_rays, half_angle_deg,
    // discretised_brush, timestep_s].
    const std::array tier{static_cast<double>(p.n_rays), p.half_angle_deg,
                          p.discretised_brush ? 1.0 : 0.0, p.timestep_s};
    vector(golden, tier, "tier");
    const std::array profile{p.profile.x0_m, p.profile.dx_m,
                             static_cast<double>(p.profile.z_m.size())};
    vector(golden, profile, "profile");
    bt::require(io::find_member(tire, "surface_map") != nullptr,
                "the track surface map rode the wire");
    // Construction on the fixture image is the projected-config gate.
    Stepper st(std::span<const std::byte>(
                       fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb")),
               cfg);
    golden.done();
}

// test_config_rejects_bad_pneumatic_sections — the ten wire mutations.
// The Python test records no golden entries (pure pytest.raises sweep);
// like the 3.12a rejection ports the contract is exercised directly.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void test_config_rejects_bad_pneumatic_sections() {
    const auto mjb = fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb");
    const auto mutate_reject = [&](const auto &mutate) {
        auto bad = io::parse_json(
                fixture_text(platform_fixture("env_fast") / "wire.json"));
        mutate(bad);
        rejects<std::exception>([&] {
            Stepper st(std::span<const std::byte>(mjb),
                       nativecfg::decode(bad));
        });
    };
    const auto object_of = [](io::Value &v) -> io::Value::Object & {
        return std::get<io::Value::Object>(v.data);
    };
    const auto member_of = [](io::Value::Object &object,
                              std::string_view key) -> io::Value & {
        for (auto &member : object)
            if (member.first == key) return member.second;
        throw std::invalid_argument("mutated wire key missing");
    };
    const auto tire_of = [&](io::Value &wire) -> io::Value & {
        return member_of(object_of(wire), "tire");
    };
    const auto pneu_of = [&](io::Value &wire) -> io::Value & {
        return member_of(object_of(tire_of(wire)), "pneumatic");
    };
    const auto erase = [&](io::Value &v, std::string_view key) {
        auto &object = object_of(v);
        object.erase(std::remove_if(object.begin(), object.end(),
                                    [&](const auto &member) {
                                        return member.first == key;
                                    }),
                     object.end());
    };
    const auto set = [&](io::Value &v, std::string_view key,
                         io::Value next) {
        member_of(object_of(v), key) = std::move(next);
    };
    const auto num = [](std::string_view lexeme) {
        return io::Value{io::Number{std::string(lexeme)}};
    };

    mutate_reject([&](io::Value &v) { erase(tire_of(v), "pneumatic"); });
    mutate_reject([&](io::Value &v) {
        set(pneu_of(v), "tier", io::Value{std::string("detailed")});
    });
    mutate_reject([&](io::Value &v) { set(pneu_of(v), "n_rays", num("33")); });
    mutate_reject([&](io::Value &v) {
        set(pneu_of(v), "half_angle_deg", num("74.0"));
    });
    mutate_reject([&](io::Value &v) {
        set(pneu_of(v), "discretised_brush", io::Value{true});
    });
    mutate_reject([&](io::Value &v) {
        set(pneu_of(v), "timestep_s", num("0.001"));
    });
    mutate_reject([&](io::Value &v) { erase(pneu_of(v), "profile"); });
    mutate_reject([&](io::Value &v) {
        set(member_of(object_of(pneu_of(v)), "profile"),
            "dx_m", num("0.0"));
    });
    mutate_reject([&](io::Value &v) {
        set(member_of(object_of(pneu_of(v)), "profile"), "z_m",
            io::Value{io::Value::Array{num("0.0")}});
    });
    mutate_reject([&](io::Value &v) { erase(tire_of(v), "surface_map"); });
}
NATIVE_DIAG_POP

// test_pneumatic_state_wire_schema_and_round_trip
void test_pneumatic_state_wire_schema_and_round_trip() {
    auto golden =
            node(kFile,
                 "test_pneumatic_state_wire_schema_and_round_trip");
    const auto mjb = fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb");
    Stepper st(std::span<const std::byte>(mjb), platform_config("env_fast"));
    const auto &names = st.tire_state_names();
    // 'names' — the applier's state_names() join, byte-compared.
    text(golden, join_lines(names), "names");
    const std::size_t n = 64;                       // fast tier rays
    const std::size_t width = (n + 1) / 2;
    bt::require(names.size() == 2 * (4 * n + 8 + 3 * width),
                "wire column count formula");
    bt::require(names[0] == "front.ray.000.delta_prev_m" &&
                        names[1] == "front.ray.000.elastic_prev_n" &&
                        names[2] == "front.ray.000.maxwell_n" &&
                        names[3] == "front.ray.000.bristle_m",
                "first four columns are the front ray rings");
    bt::require(names[4 * n] == "front.carcass.initialized" &&
                        names[4 * n + 1] == "front.rim.active" &&
                        names[4 * n + 7] == "front.patch.count" &&
                        names[4 * n + 8] == "front.patch.00.start",
                "front scalar/patch column positions");
    bt::require(names.back() == "rear.patch." +
                        (width - 1 < 10 ? std::string("0") : "") +
                        std::to_string(width - 1) + ".kappa_prime",
                "last column is the rear patch kappa_prime");

    const auto seed = golden.f64_input("input.row");
    st.set_tire_state(names, seed);
    const auto once = st.tire_state();
    vector(golden, once, "state round trip");
    st.set_tire_state(names, once);
    vector(golden, st.tire_state(), "state re-round trip");
    // The lumped fast tier leaves every bristle cell NaN.
    const auto &wire = once;
    for (std::size_t i = 0; i < n; ++i) {
        const auto tag = "front.ray." + std::to_string(i / 100) +
                std::to_string((i / 10) % 10) + std::to_string(i % 10) +
                ".bristle_m";
        bt::require(std::isnan(wire[channel(names, tag)]),
                    tag + " is NaN on the lumped tier");
    }
    golden.done();
}

// test_pneumatic_state_rejections_are_atomic
void test_pneumatic_state_rejections_are_atomic() {
    auto golden =
            node(kFile, "test_pneumatic_state_rejections_are_atomic");
    const auto mjb = fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb");
    Stepper st(std::span<const std::byte>(mjb), platform_config("env_fast"));
    const auto &names = st.tire_state_names();
    // The oracle records 'input.committed' before 'names'.
    const auto committed = golden.f64_input("input.committed");
    text(golden, join_lines(names), "names");
    st.set_tire_state(names, committed);

    rejects<std::runtime_error>(
            [&] {
                st.set_tire_state(
                        std::span<const std::string>(names)
                                .first(names.size() - 1),
                        std::span<const double>(committed)
                                .first(committed.size() - 1));
            },
            "rear.patch");
    {
        auto extra = std::vector<std::string>(names.begin(), names.end());
        extra.emplace_back("bogus");
        auto row = committed;
        row.push_back(0.0);
        rejects<std::runtime_error>(
                [&] { st.set_tire_state(extra, row); },
                "unknown column");
    }
    {
        auto bad = committed;
        bad[channel(names, "front.ray.000.delta_prev_m")] =
                std::numeric_limits<double>::infinity();
        rejects<std::runtime_error>(
                [&] { st.set_tire_state(names, bad); });
    }
    {
        // Disjointness is wire contract: an overlapping patch pair
        // rejects before any commit.
        auto overlap = committed;
        overlap[channel(names, "front.patch.count")] = 2.0;
        overlap[channel(names, "front.patch.00.start")] = 0.0;
        overlap[channel(names, "front.patch.00.stop")] = 5.0;
        overlap[channel(names, "front.patch.00.kappa_prime")] = 0.1;
        overlap[channel(names, "front.patch.01.start")] = 4.0;
        overlap[channel(names, "front.patch.01.stop")] = 8.0;
        overlap[channel(names, "front.patch.01.kappa_prime")] = 0.2;
        rejects<std::runtime_error>(
                [&] { st.set_tire_state(names, overlap); });
    }
    vector(golden, st.tire_state(),
           "rejected restores preserve the state wire");
    golden.done();
}

// test_pneumatic_qfrc_bitwise[slow] — the 40-step episode's full parity.
void test_pneumatic_qfrc_bitwise() {
    auto golden = node(kFile, "test_pneumatic_qfrc_bitwise");
    golden_parity(golden, "fast40", "env_fast");
    golden.done();
}

// The patch-bound comparison the event-rich test ends with — sorted per
// side tuple.
struct Seen {
    std::vector<bool> multi, coverage, rim, airborne;
    std::vector<std::string> surfaces;
};

// test_pneumatic_golden_episode_event_rich[slow] — STOPPED (see
// main): parked until the oracle produces two-platform goldens.
[[maybe_unused]] void test_pneumatic_golden_episode_event_rich() {
    auto golden =
            node(kFile, "test_pneumatic_golden_episode_event_rich");
    golden_parity(golden, "event280", "env_event");

    // The scripted-corpus assertions run on the artifact itself — the
    // manifest diagnostics the native side matched bitwise at each step.
    auto ep = load_episode_bundle("event280");
    const auto steps = ep.artifact.state_qpos.shape.at(0);
    bt::require(steps == 280, "event episode is 280 rows");
    const auto &diag_list = io::as_array(
            *io::find_member(io::as_object(ep.artifact.manifest,
                                         "manifest"),
                             "tire_diagnostics"),
            "tire_diagnostics");

    auto field = [&](std::size_t k, std::string_view side,
                     std::string_view key) -> const io::Value & {
        return *io::find_member(
                io::as_object(
                        *io::find_member(
                                io::as_object(diag_list[k],
                                              "tire_diagnostics[k]"),
                                std::string(side)),
                        "diag side"),
                std::string(key));
    };
    auto boolv = [&](std::size_t k, std::string_view side,
                     std::string_view key) {
        return io::as_bool(field(k, side, key), "diag bool");
    };
    auto i64v = [&](std::size_t k, std::string_view side,
                    std::string_view key) {
        return io::as_int64(field(k, side, key), "diag int");
    };

    Seen front, rear;
    for (std::size_t k = 0; k < steps; ++k)
        for (auto *seen : {&front, &rear}) {
            const std::string_view side =
                    seen == &front ? "front" : "rear";
            seen->multi.push_back(i64v(k, side, "patch_count") > 1);
            seen->coverage.push_back(boolv(k, side, "coverage_event"));
            seen->rim.push_back(boolv(k, side, "rim_strike_active"));
            seen->airborne.push_back(boolv(k, side, "airborne"));
            seen->surfaces.push_back(
                    io::as_string(field(k, side, "surface"), "surface"));
        }

    const auto air_runs = [](const std::vector<bool> &flags) {
        std::size_t runs = 0, longest = 0, current = 0;
        for (const bool flag : flags) {
            current = flag ? current + 1 : 0;
            if (flag && current == 1) ++runs;
            longest = std::max(longest, current);
        }
        return std::pair{runs, longest};
    };
    const auto [front_runs, front_longest] = air_runs(front.airborne);
    bt::require(front_runs >= 2 && front_longest >= 30,
                "bump launch + nose-stand hang");
    bt::require(std::ranges::count(rear.airborne, true) >= 10,
                "rear airborne on the ledge pitch");
    bt::require(std::ranges::count(front.coverage, true) >= 5,
                "coverage gate on the step face");
    bt::require(std::ranges::count(front.rim, true) >= 15,
                "rim strikes on the face");
    bt::require(std::ranges::count(front.multi, true) >= 3,
                "bump lip splits the front patch");
    for (const auto &[seen, minimum] :
         {std::pair{&front, 1U}, {&rear, 0U}}) {
        std::size_t transitions = 0;
        for (std::size_t k = 1; k < seen->surfaces.size(); ++k)
            transitions += seen->surfaces[k] != seen->surfaces[k - 1];
        // Python requires >=2 front / >=1 rear.
        bt::require(transitions >= minimum + 1,
                    "surface transitions ride the diagnostics");
    }

    // The consumed patch (start, stop) bounds migrate as the contact
    // evolves — every drift through the wire counts.
    std::size_t drift = 0;
    for (std::size_t k = 1; k < steps; ++k)
        for (std::string_view side : {"front", "rear"})
            drift += patch_bounds(row(ep.tire_state, k, ep.wire),
                                  ep.artifact.tire_state_names, side) !=
                     patch_bounds(row(ep.tire_state, k - 1, ep.wire),
                                  ep.artifact.tire_state_names, side);
    bt::require(drift >= 60, "patch bounds drifted");
    const auto rim_energy =
            channel(ep.artifact.tire_state_names, "front.rim.energy_j");
    std::size_t finite = 0;
    for (std::size_t k = 0; k < steps; ++k)
        finite += std::isfinite(
                row(ep.tire_state, k, ep.wire)[rim_energy]);
    bt::require(finite >= 10, "rim-event record rode the wire");
    golden.done();
}

// test_pneumatic_golden_reset_restore_mid_run[slow]
void test_pneumatic_golden_reset_restore_mid_run() {
    auto golden =
            node(kFile, "test_pneumatic_golden_reset_restore_mid_run");
    auto ep = load_episode_bundle("fast40");
    const auto mjb = fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb");
    Stepper st(std::span<const std::byte>(mjb), platform_config("env_fast"));
    // No 'names'/'force_names' records in this test — the stream starts
    // at 'tires golden step 0'.
    const double dt = st.model()->opt.timestep;
    constexpr std::size_t mid = 15, rows = 30;
    for (std::size_t k = 0; k < rows; ++k) {
        const std::string tag = "step " + std::to_string(k);
        if (k == mid) st.reset();
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na),
                     row(ep.warmstart, k, ep.nv), ep.time[k]);
        st.forward();
        st.set_tire_state(ep.artifact.tire_state_names,
                          row(ep.tire_state, k, ep.wire));
        const auto got = st.tire_qfrc(dt);
        vector(golden, got, "tires golden step " + std::to_string(k));
        vector(golden, got,
               "tires live-oracle step " + std::to_string(k));
        vector(golden, st.tire_state(),
               "tire_state evolve step " + std::to_string(k));
        snapshots(golden, st, tag);
        diagnostics(golden, st, tag);
    }
    golden.done();
}

// test_pneumatic_golden_failure_atomicity[slow]
void test_pneumatic_golden_failure_atomicity() {
    auto golden =
            node(kFile, "test_pneumatic_golden_failure_atomicity");
    auto ep = load_episode_bundle("fast40");
    const auto mjb = fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb");
    Stepper st(std::span<const std::byte>(mjb), platform_config("env_fast"));
    const auto &names = st.tire_state_names();
    const double dt = st.model()->opt.timestep;
    constexpr std::size_t mid = 12, rows = 24;

    const auto replay = [&](std::size_t k) {
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na),
                     row(ep.warmstart, k, ep.nv), ep.time[k]);
        st.forward();
        st.set_tire_state(names, row(ep.tire_state, k, ep.wire));
        vector(golden, st.tire_qfrc(dt),
               "tires golden step " + std::to_string(k));
    };
    for (std::size_t k = 0; k < mid; ++k) replay(k);

    const auto committed = st.tire_state();

    // Poisoned inputs leave every committed artifact untouched: the
    // compares below replay the same label stream the oracle emitted —
    // the frozen copies are the oracle's expected side; the C++ emits
    // the live post-rejection views.
    const auto assert_untouched = [&](const char *label) {
        vector(golden, st.tire_state(),
               std::string(label) + ": state wire");
        // The oracle helpers wrap k in 'step {k}' — with k = the label.
        snapshots(golden, st, std::string("step ") + label);
        diagnostics(golden, st, std::string("step ") + label);
        scalar(golden, st.tire_stored_energy(),
               std::string(label) + ": stored_energy");
        scalar(golden, st.tire_brush_loss_step_j(),
               std::string(label) + ": brush_loss_step_j");
        scalar(golden, st.tire_radial_dissipation_power_w(),
               std::string(label) + ": radial_dissipation_power_w");
    };

    // A non-finite state cell: the rejection precedes any commit.
    {
        auto bad = committed;
        bad[channel(names, "front.ray.000.delta_prev_m")] =
                std::numeric_limits<double>::infinity();
        rejects<std::runtime_error>([&] { st.set_tire_state(names, bad); });
        assert_untouched("poisoned wire");
    }
    // The gated compute rejects before the wheel loop.
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(-dt); });
    assert_untouched("negative dt");
    rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                   "increasing timestamp");
    assert_untouched("repeated timestamp");
    for (std::size_t k = mid; k < rows; ++k) replay(k);
    // _assert_*_equal(..., 'final step') — the helpers wrap k in
    // 'step {k}', producing '... step final step' labels.
    snapshots(golden, st, "step final step");
    diagnostics(golden, st, "step final step");
    golden.done();
}

// ---- the detailed-tier small rig ------------------------------------

// Split one packed 'input.state {k}' row (concat of qpos, qvel, act,
// warmstart, [time]) against the stepper's model widths and replay it.
struct PackedRow {
    std::vector<double> qpos, qvel, act, warmstart;
    double time = 0.;
};

[[nodiscard]] PackedRow unpack_state(const Stepper &st,
                                     std::span<const double> packed) {
    const mjModel *m = st.model();
    const std::size_t nq = static_cast<std::size_t>(m->nq),
            nv = static_cast<std::size_t>(m->nv),
            na = static_cast<std::size_t>(m->na);
    bt::require(packed.size() == nq + nv + na + nv + 1,
                "input.state row width");
    PackedRow row;
    auto cursor = packed.begin();
    const auto take = [&](std::size_t width) {
        return std::vector<double>(
                cursor, cursor + static_cast<std::ptrdiff_t>(width));
    };
    row.qpos = take(nq);
    cursor += static_cast<std::ptrdiff_t>(nq);
    row.qvel = take(nv);
    cursor += static_cast<std::ptrdiff_t>(nv);
    row.act = take(na);
    cursor += static_cast<std::ptrdiff_t>(na);
    row.warmstart = take(nv);
    cursor += static_cast<std::ptrdiff_t>(nv);
    row.time = packed.back();
    return row;
}

void scripted_tick(bt::oracle::Node &golden, Stepper &st, std::size_t k,
                   const std::string &qfrc_label,
                   const std::string &wire_label,
                   const std::vector<std::string> &names) {
    // The oracle records the consumed seed row before the packed state.
    const auto seed = golden.f64_input("input.seed " + std::to_string(k));
    const auto packed = golden.f64_input("input.state " + std::to_string(k));
    const auto row = unpack_state(st, packed);
    st.set_state(row.qpos, row.qvel, row.act, row.warmstart, row.time);
    st.forward();
    st.set_tire_state(names, seed);
    const double dt = st.model()->opt.timestep;
    vector(golden, st.tire_qfrc(dt), qfrc_label);
    vector(golden, st.tire_state(), wire_label);
}

// test_pneumatic_qfrc_bitwise_detailed_tier
void test_pneumatic_qfrc_bitwise_detailed_tier() {
    auto golden =
            node(kFile, "test_pneumatic_qfrc_bitwise_detailed_tier");
    const auto mjb =
            fixture_bytes(fixture("rig_detailed") / "model.mjb");
    Stepper st(std::span<const std::byte>(mjb),
               load_config("rig_detailed"));
    const auto &names = st.tire_state_names();
    text(golden, join_lines(names), "names");
    const std::size_t n = 256;
    const std::size_t width = (n + 1) / 2;
    bt::require(names.size() == 2 * (4 * n + 8 + 3 * width),
                "detailed wire column count formula");

    for (std::size_t k = 0; k < 12; ++k) {
        const std::string tick = "tick " + std::to_string(k);
        scripted_tick(golden, st, k,
                      "detailed qfrc " + tick,
                      "detailed evolved wire " + std::to_string(k),
                      {names.begin(), names.end()});
        snapshots(golden, st, tick);
        diagnostics(golden, st, "step " + std::to_string(k));
        scalar(golden, st.tire_stored_energy(),
               "detailed stored_energy " + tick);
        scalar(golden, st.tire_brush_loss_step_j(),
               "detailed brush_loss " + tick);
        scalar(golden, st.tire_radial_dissipation_power_w(),
               "detailed radial_power " + tick);
    }
    // Bristle cells engaged: finite inside committed patches, NaN out.
    const auto wire = st.tire_state();
    std::size_t finite = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto tag = "front.ray." + std::to_string(i / 100) +
                std::to_string((i / 10) % 10) + std::to_string(i % 10) +
                ".bristle_m";
        finite += std::isfinite(wire[channel(names, tag)]);
    }
    bt::require(finite > 0 && finite < n,
                "bristle wire partially engaged on the detailed tier");
    golden.done();
}

// test_pneumatic_rig_event_sweep_detailed_tier
void test_pneumatic_rig_event_sweep_detailed_tier() {
    auto golden =
            node(kFile,
                 "test_pneumatic_rig_event_sweep_detailed_tier");
    const auto mjb = fixture_bytes(fixture("rig_events") / "model.mjb");
    Stepper st(std::span<const std::byte>(mjb), load_config("rig_events"));
    const auto &names = st.tire_state_names();
    text(golden, join_lines(names), "names");
    static const std::array<std::string_view, 15> labels{
            "flat",     "notch-appr", "notch-F",   "notch-R",
            "hold",     "scree-both", "wet-F",     "cliff-1",
            "cliff-2",  "cliff-3",    "cliff-4",   "cliff-face",
            "airborne", "flat-rim",   "rear-cliff"};

    std::map<std::string, Seen> seen{{"front", {}}, {"rear", {}}};
    std::vector<std::pair<std::vector<std::pair<int, int>>,
                          std::vector<std::pair<int, int>>>>
            wire_bounds;
    const auto diags_of = [&] { return st.tire_writer().diagnostics(); };
    for (std::size_t k = 0; k < 15; ++k) {
        const std::string label(labels.at(k));
        scripted_tick(golden, st, k, label + ": qfrc",
                      label + ": evolved wire",
                      {names.begin(), names.end()});
        snapshots(golden, st, "tick " + label);
        diagnostics(golden, st, "step " + label);
        scalar(golden, st.tire_stored_energy(),
               label + ": stored_energy");
        scalar(golden, st.tire_brush_loss_step_j(), label + ": brush_loss");
        scalar(golden, st.tire_radial_dissipation_power_w(),
               label + ": radial_power");
        const auto wire = st.tire_state();
        wire_bounds.emplace_back(
                patch_bounds(wire, {names.begin(), names.end()}, "front"),
                patch_bounds(wire, {names.begin(), names.end()}, "rear"));
        const auto diags = diags_of();
        for (int side = 0; side < 2; ++side) {
            auto &s = seen[side == 0 ? "front" : "rear"];
            const auto &d = *diags[static_cast<std::size_t>(side)];
            s.multi.push_back(d.patch_count > 1);
            s.coverage.push_back(d.coverage_event);
            s.rim.push_back(d.rim_strike_active);
            s.airborne.push_back(d.airborne);
            s.surfaces.push_back(d.surface);
        }
    }
    bt::require(std::ranges::any_of(seen["front"].multi,
                                    [](bool v) { return v; }) &&
                        std::ranges::any_of(seen["rear"].multi,
                                            [](bool v) { return v; }),
                "multi-patch on both wheels");
    bt::require(std::ranges::count(seen["front"].coverage, true) >= 4,
                "coverage >= 4");
    bt::require(std::ranges::any_of(seen["rear"].coverage,
                                    [](bool v) { return v; }) &&
                        std::ranges::any_of(seen["rear"].rim,
                                            [](bool v) { return v; }),
                "rear coverage and rim");
    bt::require(std::ranges::count(seen["front"].rim, true) >= 5,
                "front rim strikes");
    bt::require(std::ranges::any_of(seen["front"].airborne,
                                    [](bool v) { return v; }),
                "front airborne tick");
    for (const char *side : {"front", "rear"}) {
        const auto &surfaces = seen[side].surfaces;
        std::size_t transitions = 0;
        for (std::size_t i = 1; i < surfaces.size(); ++i)
            transitions += surfaces[i] != surfaces[i - 1];
        bt::require(transitions >= 1,
                    std::string(side) + " surface transitions");
    }
    std::size_t drift = 0;
    for (std::size_t k = 1; k < wire_bounds.size(); ++k)
        drift += wire_bounds[k] != wire_bounds[k - 1];
    bt::require(drift >= 10, "patch bounds migrated across the sweep");
    golden.done();
}

// test_pneumatic_probe_and_clock — the Stepper clock half replays row 2
// of the five-step episode; the runtime half drives a NativeRideRuntime
// on the prepared env_fast bootstrap.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[maybe_unused]] void test_pneumatic_probe_and_clock() {
    auto golden = node(kFile, "test_pneumatic_probe_and_clock");
    auto ep = load_episode_bundle("fast40");
    const auto mjb = fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb");
    {
        Stepper st(std::span<const std::byte>(mjb),
                   platform_config("env_fast"));
        const auto &names = st.tire_state_names();
        text(golden, join_lines(names), "names");
        const double dt = st.model()->opt.timestep;
        constexpr std::size_t k = 2;
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na),
                     row(ep.warmstart, k, ep.nv), ep.time[k]);
        st.forward();
        st.set_tire_state(names, row(ep.tire_state, k, ep.wire));
        (void)st.tire_qfrc(dt);
        rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                       "increasing timestamp");
        const auto committed = st.tire_state();
        {
            auto bad = committed;
            bad[channel({names.begin(), names.end()},
                        "front.ray.000.delta_prev_m")] =
                    std::numeric_limits<double>::infinity();
            rejects<std::runtime_error>(
                    [&] { st.set_tire_state(names, bad); });
        }
        vector(golden, st.tire_state(),
               "rejected set_state preserves the state wire");
        rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                       "increasing timestamp");
        st.set_tire_state(names, committed);  // a valid restore re-arms
        (void)st.tire_qfrc(dt);
        rejects<std::invalid_argument>([&] { (void)st.tire_qfrc(dt); },
                                       "increasing timestamp");
    }

    // The runtime probe half: NativeRideRuntime on the staged bootstrap,
    // record_decimation pinned to the oracle's 1.
    auto request = bike::research::read_research_request(
            platform_fixture("env_fast") / "request.toml");
    request.experiment.record_decimation = 1;
    auto prepared = bike::research::prepare_environment(request);
    ::runtime::NativeRideRuntime native(std::move(prepared.bootstrap.ride));
    const ::runtime::RideControl command{};
    const auto period = static_cast<std::int64_t>(
            golden.f64_input("input.period").at(0));
    for (std::int64_t target = 1; target <= period; ++target) {
        (void)native.advance(target, command, 0., 0., std::nullopt);
        vector(golden, native.snapshot().integration_state,
               "interval " + std::to_string(target));
    }
    const auto before = native.snapshot();
    const auto probe = native.probe_step_inputs(command, 0., 0.);
    {
        std::string joined;
        for (std::size_t i = 0; i < probe.components.size(); ++i) {
            if (i) joined.push_back('\n');
            joined += probe.components[i].first;
        }
        text(golden, joined, "probe component names");
        for (const auto &[name, force] : probe.components) {
            text(golden, name, "probe name " + name);
            vector(golden, force, "probe " + name);
        }
    }
    const auto after = native.snapshot();
    count(golden, after.step, "probe after.step");
    boolean(golden, before.latest_sample != nullptr, "probe has_sample");
    count(golden, before.latest_sample->interval_id, "probe interval_id");
    bt::require(after.step == before.step && before.step == period,
                "probe published nothing");
    (void)native.advance(period + 1, command, 0., 0., std::nullopt);
    vector(golden, native.snapshot().integration_state,
           "post-probe interval");
    native.close();
    golden.done();
}
NATIVE_DIAG_POP

// test_pneumatic_observations_declare_multi_contact — the committed
// 'tires' channel on the research runtime's first step.
void test_pneumatic_observations_declare_multi_contact() {
    auto golden = node(
            kFile, "test_pneumatic_observations_declare_multi_contact");
    auto request = bike::research::read_research_request(
            platform_fixture("env_fast") / "request.toml");
    request.experiment.duration_s = .05;
    auto native = bike::research::make_environment(request);
    const ::runtime::RideControl command{};
    native->begin_control(command, 0., 0.);
    (void)native->advance_control(std::nullopt);
    native->acknowledge_control();
    const auto snap = native->snapshot();
    bt::require(snap.latest_sample != nullptr, "a committed sample");
    const auto *tires = [&]() -> const ::runtime::Wire * {
        for (const auto &[name, wire] : snap.latest_sample->channels)
            if (name == "tires") return &wire;
        return nullptr;
    }();
    bt::require(tires != nullptr, "tires channel published");
    const auto &channel_obj = std::get<::runtime::WireObject>(tires->value);
    for (const char *side : {"front", "rear"}) {
        const ::runtime::Wire *entry = nullptr;
        for (const auto &[name, wire] : channel_obj)
            if (name == side) entry = &wire;
        bt::require(entry != nullptr, std::string(side) + " entry");
        const auto &dict =
                std::get<::runtime::WireObject>(entry->value);
        const auto member = [&](std::string_view key) -> const ::runtime::Wire & {
            for (const auto &[name, wire] : dict)
                if (name == key) return wire;
            throw std::invalid_argument("channel key missing");
        };
        text(golden, std::get<std::string>(member("backend").value),
             std::string(side) + ".backend");
        boolean(golden,
                std::get<bool>(
                        member("supports_multiple_contacts").value),
                std::string(side) + ".supports_multiple_contacts");
        scalar(golden,
               std::get<double>(member("normal_load_n").value),
               std::string(side) + ".normal_load_n");
        count(golden, std::get<std::int64_t>(member("patch_count").value),
              std::string(side) + ".patch_count");
        text(golden, std::get<std::string>(member("surface").value),
             std::string(side) + ".surface");
    }
    native->close();
    golden.done();
}

// test_pneumatic_stored_energy_and_counters — every fifth row of the
// first 20 steps.
void test_pneumatic_stored_energy_and_counters() {
    auto golden =
            node(kFile, "test_pneumatic_stored_energy_and_counters");
    auto ep = load_episode_bundle("fast40");
    const auto mjb = fixture_bytes(platform_fixture("episodes") / "fast40" / "model.mjb");
    Stepper st(std::span<const std::byte>(mjb), platform_config("env_fast"));
    const auto &names = st.tire_state_names();
    const double dt = st.model()->opt.timestep;
    for (std::size_t k = 0; k < 20; k += 5) {
        st.set_state(row(ep.qpos, k, ep.nq), row(ep.qvel, k, ep.nv),
                     row(ep.act, k, ep.na),
                     row(ep.warmstart, k, ep.nv), ep.time[k]);
        st.forward();
        st.set_tire_state(names, row(ep.tire_state, k, ep.wire));
        (void)st.tire_qfrc(dt);
        const std::string tag = "step " + std::to_string(k);
        scalar(golden, st.tire_stored_energy(), "stored_energy " + tag);
        scalar(golden, st.tire_brush_loss_step_j(),
               "brush_loss_step_j " + tag);
        scalar(golden, st.tire_radial_dissipation_power_w(),
               "radial_dissipation_power_w " + tag);
    }
    golden.done();
}

} // namespace

// cppcheck-suppress throwInEntryPoint
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_config_projection_carries_the_pneumatic_section();
        test_config_rejects_bad_pneumatic_sections();
        test_pneumatic_state_wire_schema_and_round_trip();
        test_pneumatic_state_rejections_are_atomic();
        test_pneumatic_qfrc_bitwise();
        // STOPPED (D16): test_pneumatic_golden_episode_event_rich and
        // test_pneumatic_probe_and_clock are baseline-red in the Python
        // oracle on this lane — the event env's physical equilibrium does
        // not converge (64000-step budget, front_wheel_spin residual
        // ~0.081 — the documented savage-plant defect family) and the
        // probe interval diverges bitwise from the live sim at step 1.
        // Both verified identical without the capture edits. Their C++
        // ports are authored below and wired in once the oracle is green
        // and both platforms' goldens exist.
        // test_pneumatic_golden_episode_event_rich();
        test_pneumatic_golden_reset_restore_mid_run();
        test_pneumatic_golden_failure_atomicity();
        test_pneumatic_qfrc_bitwise_detailed_tier();
        test_pneumatic_rig_event_sweep_detailed_tier();
        // test_pneumatic_probe_and_clock();
        test_pneumatic_observations_declare_multi_contact();
        test_pneumatic_stored_energy_and_counters();
    });
}
NATIVE_DIAG_POP
