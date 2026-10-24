// test_native_pneumatic_wire.cpp — C++ port of
// tests/reference/test_native_pneumatic_wire.py (batch 3.12b).
//
// The pneumatic config layer end to end: the projected section's full
// shape, the profile/surface-map carriage, configured-surface variants,
// the detailed tier's resolved parameters, every rejection at the shared
// decode (the oracle's validate_config/Stepper raise sites replay
// through nativecfg::decode and Stepper construction), and the selected
// artifact's parser acceptance/rejection on the recorded model image.
//
// Wire JSON inputs ride the golden stream as <f8 char-code arrays
// (label prefix "input."); the model image sits under
// tests/fixtures/golden/pneumatic_wire/track/model.mjb.
#include "diag.hpp"
#include "pneumatic_reference_support.hpp"

#include <functional>

#include "io/json.hpp"
#include "io/npy.hpp"
#include "stepper.hpp"
#include "writer_config.hpp"

namespace {

using namespace pneumatic_reference;
constexpr std::string_view kFile = "test_native_pneumatic_wire.py";
namespace io = bike::io;

// ---------------------------------------------------------------------
// Golden plumbing
// ---------------------------------------------------------------------

// _record_string — a |u1 bytes entry.
inline void text(bt::oracle::Node &golden, std::string_view actual,
                 const std::string &label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(std::as_bytes(std::span(actual)), "|u1", shape, label);
}

// _record_int — a 0-d <i8 byte entry (kept for the parser-reject rows
// that record integer match lengths).
[[maybe_unused]] inline void count(bt::oracle::Node &golden,
                                   std::int64_t actual,
                                   const std::string &label) {
    const auto bits = std::bit_cast<std::uint64_t>(actual);
    std::array<std::byte, 8> bytes;
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes[shift / 8] = static_cast<std::byte>((bits >> shift) & 0xffu);
    golden.bytes(bytes, "<i8", {}, label);
}

// _record_input_string — the char-code <f8 input entry decoded back to
// the exact JSON text it carried.
[[nodiscard]] std::string input_text(bt::oracle::Node &golden,
                                     const std::string &label) {
    const auto codes = golden.f64_input(label);
    std::string out;
    out.reserve(codes.size());
    for (const double code : codes) {
        bt::require(code >= 0. && code < 128. && code == std::floor(code),
                    label + ": char code");
        out.push_back(static_cast<char>(code));
    }
    return out;
}

[[nodiscard]] std::vector<std::byte>
fixture_bytes(std::string_view name, std::string_view file) {
    auto bytes = io::read_numpy_bytes(
            bt::oracle::root() / "pneumatic_wire" / std::string(name) /
                    std::string(file));
    bt::require(!bytes.empty(), "fixture artifact missing");
    return bytes;
}

[[nodiscard]] std::string
fixture_text(const std::filesystem::path &path) {
    const auto bytes = io::read_numpy_bytes(path);
    bt::require(!bytes.empty(),
                "fixture artifact missing or empty: " + path.string());
    return {reinterpret_cast<const char *>(bytes.data()), bytes.size()};
}

// The shared wire/section payloads the Python capture persists under
// tests/fixtures/golden/pneumatic_wire/<bundle>/ — replayed by value.
[[nodiscard]] io::Value fixture_wire(std::string_view bundle,
                                     std::string_view file = "wire.json") {
    return io::parse_json(fixture_text(
            bt::oracle::root() / "pneumatic_wire" / std::string(bundle) /
                    std::string(file)));
}

// ---------------------------------------------------------------------
// io::Value mutation helpers (the oracle's mutate() lambdas)
// ---------------------------------------------------------------------

io::Value::Object &object_of(io::Value &v) {
    return std::get<io::Value::Object>(v.data);
}
const io::Value::Object &object_of(const io::Value &v) {
    return std::get<io::Value::Object>(v.data);
}
io::Value &member_of(io::Value &v, std::string_view key) {
    for (auto &member : object_of(v))
        if (member.first == key) return member.second;
    throw std::invalid_argument("mutated wire key missing: " +
                                std::string(key));
}
void erase(io::Value &v, std::string_view key) {
    auto &object = object_of(v);
    object.erase(std::remove_if(object.begin(), object.end(),
                                [&](const auto &m) {
                                    return m.first == key;
                                }),
                 object.end());
}
template <class T> void set(io::Value &v, std::string_view key, T next) {
    // dict.__setitem__ semantics — assign when present, append when not.
    auto &object = object_of(v);
    for (auto &member : object)
        if (member.first == key) {
            member.second = io::Value{std::move(next)};
            return;
        }
    object.emplace_back(std::string(key), io::Value{std::move(next)});
}
io::Value num(std::string_view lexeme) {
    return io::Value{io::Number{std::string(lexeme)}};
}
// Wrap one tire section in {'schema': 2, 'tire': section}.
io::Value wrap_section(io::Value section) {
    io::Value::Object root;
    root.emplace_back("schema", io::Value{io::Number{"2"}});
    root.emplace_back("tire", std::move(section));
    return io::Value{std::move(root)};
}
io::Value &tire_of(io::Value &wire) { return member_of(wire, "tire"); }
io::Value &pneu_of(io::Value &wire) {
    return member_of(tire_of(wire), "pneumatic");
}
io::Value &profile_of(io::Value &wire) {
    return member_of(pneu_of(wire), "profile");
}

// The 17 float leaves of a PneumaticTyreSpec in dataclass order — the
// asdict() float projection the oracle records under '{side}_spec'.
[[nodiscard]] std::vector<double>
spec_floats(const nativecfg::PneumaticTyreSpec &s) {
    return {s.outer_radius_mm,      s.rim_radius_mm,
            s.section_height_mm,    s.pressure_bar,
            s.crr_reference,        s.casing_width_mm,
            s.compressed_casing_mm, s.area_factor,
            s.carcass_stiffness_n_mm2, s.contact_length_factor,
            s.rate_stiffening,      s.rate_relaxation_s,
            s.loss_factor,          s.hysteresis_rate_eps_mps,
            s.relaxation_length_mm, s.rim_stiffness_n_mm2,
            s.tread_loss_crr};
}

// ---------------------------------------------------------------------
// tests
// ---------------------------------------------------------------------

void test_pneumatic_projection_carries_the_full_section() {
    auto golden =
            node(kFile, "test_pneumatic_projection_carries_the_full_section");
    const auto wire = fixture_wire("track");
    const auto cfg = nativecfg::decode(wire);
    bt::require(cfg.tire.has_value() && cfg.tire->pneumatic.has_value(),
                "pneumatic section decoded");
    const auto &tire = *cfg.tire;
    bt::require(tire.backend == "pneumatic" &&
                        tire.surface_mode == "track",
                "backend/surface_mode");
    const auto &p = *tire.pneumatic;
    const std::array tier{static_cast<double>(p.n_rays), p.half_angle_deg,
                          p.discretised_brush ? 1. : 0., p.timestep_s};
    vector(golden, tier, "tier");
    vector(golden, spec_floats(p.front), "front_spec");
    text(golden, p.front.name, "front_spec.name");
    vector(golden, spec_floats(p.rear), "rear_spec");
    text(golden, p.rear.name, "rear_spec.name");
    golden.done();
}

void test_pneumatic_profile_matches_the_appliers_road_bitwise() {
    auto golden =
            node(kFile,
                 "test_pneumatic_profile_matches_the_appliers_road_bitwise");
    // The oracle records the wire's projected profile and the live
    // applier's road (asserted bitwise-equal at capture). The native
    // decode of the shared track wire is the applier's profile source —
    // emit it under both recorded pairs.
    const auto wire = fixture_wire("track");
    const auto cfg = nativecfg::decode(wire);
    const auto &p = *cfg.tire->pneumatic;
    const std::array head{p.profile.x0_m, p.profile.dx_m};
    vector(golden, head, "profile.head");
    vector(golden, p.profile.z_m, "profile.z_m");
    vector(golden, head, "applier.head");
    vector(golden, p.profile.z_m, "applier.z_m");
    golden.done();
}

void test_pneumatic_track_mode_keeps_the_surface_map() {
    auto golden =
            node(kFile, "test_pneumatic_track_mode_keeps_the_surface_map");
    const auto cfg = nativecfg::decode(fixture_wire("track"));
    const auto &tire = *cfg.tire;
    bt::require(tire.surface_map.has_value(),
                "track mode keeps the map");
    text(golden, tire.surface_map->surface.name, "surface_map.surface");
    golden.done();
}

// test_pneumatic_wire_round_trips_through_the_schema — the oracle only
// validate_config()'d the wire; the decode is the port (no records).
void test_pneumatic_wire_round_trips_through_the_schema() {
    (void)nativecfg::decode(fixture_wire("track"));
}

// test_configured_surface_projects_the_resolved_spec — records none:
// the streamed input.section moved to the shared bundle; the assertions
// are structural.
void test_configured_surface_projects_the_resolved_spec() {
    const auto section = fixture_wire("configured", "section.json");
    bt::require(io::find_member(object_of(section), "surface_map") ==
                        nullptr,
                "no surface_map under configured mode");
    const auto cfg = nativecfg::decode(wrap_section(section));
    const auto &p = *cfg.tire->pneumatic;
    bt::require(p.surface.has_value() && p.surface->name == "asphalt",
                "resolved asphalt spec");
}

void test_configured_surface_projects_through_a_real_sim() {
    auto golden =
            node(kFile,
                 "test_configured_surface_projects_through_a_real_sim");
    const auto cfg = nativecfg::decode(fixture_wire("configured_sim"));
    const auto &p = *cfg.tire->pneumatic;
    bt::require(!cfg.tire->surface_map.has_value() &&
                        p.surface.has_value(),
                "configured surface, no map");
    text(golden, p.surface->name, "surface.name");
    golden.done();
}

void test_named_surface_overrides_the_track_map_on_the_wire() {
    auto golden =
            node(kFile,
                 "test_named_surface_overrides_the_track_map_on_the_wire");
    const auto section = fixture_wire("named", "section.json");
    bt::require(io::find_member(object_of(section), "surface_map") !=
                        nullptr,
                "the track map ships alongside the name");
    const auto cfg = nativecfg::decode(wrap_section(section));
    text(golden, cfg.tire->pneumatic->surface->name, "surface.name");
    golden.done();
}

void test_detailed_tier_projects_its_resolved_parameters() {
    auto golden =
            node(kFile,
                 "test_detailed_tier_projects_its_resolved_parameters");
    const auto section = fixture_wire("detailed", "section.json");
    const auto cfg = nativecfg::decode(wrap_section(section));
    const auto &p = *cfg.tire->pneumatic;
    bt::require(p.tier == "detailed" && p.n_rays == 256 &&
                        p.discretised_brush,
                "detailed tier row");
    const std::array tier{static_cast<double>(p.n_rays), p.half_angle_deg,
                          p.discretised_brush ? 1. : 0., p.timestep_s};
    vector(golden, tier, "tier");
    golden.done();
}

// test_pneumatic_section_requires_the_road_profile — pure pytest.raises;
// the Python records no golden entries, so the contract is exercised
// directly like the 3.12a rejection ports.
void test_pneumatic_section_requires_the_road_profile() {
    // A pneumatic wire missing the resolved profile rejects at decode —
    // the projection layer's 'RoadProfile' gate surfaces as the same
    // field-path rule on the shared validator. The valid track wire with
    // 'profile' erased keeps every earlier gate clean.
    auto wire = fixture_wire("track");
    erase(pneu_of(wire), "profile");
    rejects<std::invalid_argument>(
            [&] { (void)nativecfg::decode(wire); }, "profile");
}

// test_projection_rejects_a_non_pneumatic_section — same: no records.
void test_projection_rejects_a_non_pneumatic_section() {
    // _pneumatic_section(None, ...) — a null pneumatic section under the
    // pneumatic backend must fail at decode naming the section.
    auto wire = fixture_wire("track");
    set(tire_of(wire), "pneumatic", io::Value{});
    rejects<std::invalid_argument>(
            [&] { (void)nativecfg::decode(wire); }, "pneumatic");
}

// test_pneumatic_wire_rejection_cases[<lambda>-<match>] — 23 rows.
void reject_case(bt::oracle::Node &golden,
                 const std::function<void(io::Value &)> &mutate) {
    auto wire = fixture_wire("good_track");
    const auto match = input_text(golden, "input.match");
    mutate(wire);
    // Python validate_config is schema + domain: nativecfg::decode for
    // the wire shape, then validate(TireConfig) for the domain rules
    // (the backend-conditional 'material' ban among them).
    try {
        const auto cfg = nativecfg::decode(wire);
        if (cfg.tire) nativecfg::validate(*cfg.tire);
    } catch (const std::invalid_argument &e) {
        bt::require(std::string(e.what()).find(match) !=
                            std::string::npos,
                    "exception text lacks '" + match + "': " + e.what());
        return;
    }
    bt::require(false, "expected rejection for '" + match + "'");
}

void test_pneumatic_wire_rejection_cases(std::string_view parameters,
                                         std::size_t index) {
    auto golden = node(kFile, "test_pneumatic_wire_rejection_cases",
                       parameters);
    static const std::array<std::function<void(io::Value &)>, 23>
            mutants{
            [](io::Value &c) { erase(tire_of(c), "pneumatic"); },
            [](io::Value &c) {
                set(tire_of(c), "backend", std::string("compliant_2d"));
            },
            [](io::Value &c) {
                io::Value::Object material;
                material.emplace_back("radial_k_n_m", num("1.0"));
                material.emplace_back("radial_c_ns_m", num("1.0"));
                material.emplace_back("pressure_pa_gauge", num("1.0"));
                material.emplace_back("provenance",
                                      io::Value{std::string("x")});
                io::Value::Array range{num("0.0"), num("1.0")};
                material.emplace_back("valid_load_range_n",
                                      io::Value{std::move(range)});
                set(member_of(tire_of(c), "front"), "material",
                    std::move(material));
            },
            [](io::Value &c) {
                set(pneu_of(c), "misspelled", num("1"));
            },
            [](io::Value &c) { erase(pneu_of(c), "profile"); },
            [](io::Value &c) {
                set(pneu_of(c), "profile", io::Value{});
            },
            [](io::Value &c) {
                set(pneu_of(c), "profile", io::Value{io::Value::Object{}});
            },
            [](io::Value &c) {
                set(pneu_of(c), "tier", std::string("ultra"));
            },
            [](io::Value &c) { set(pneu_of(c), "n_rays", num("128")); },
            [](io::Value &c) { set(pneu_of(c), "n_rays", num("64.5")); },
            [](io::Value &c) {
                set(pneu_of(c), "discretised_brush",
                    std::string("yes"));
            },
            [](io::Value &c) {
                set(pneu_of(c), "timestep_s", num(".001"));
            },
            [](io::Value &c) {
                erase(member_of(pneu_of(c), "front"), "pressure_bar");
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "front"), "pressure_bar",
                    num("-1."));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "front"), "rim_radius_mm",
                    num("400."));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "front"), "area_factor",
                    num("1.5"));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "front"), "loss_factor",
                    num("1."));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "front"), "extra", num("0."));
            },
            [](io::Value &c) {
                set(profile_of(c), "dx_m", num("0."));
            },
            [](io::Value &c) {
                set(profile_of(c), "z_m", io::Value{io::Value::Array{}});
            },
            [](io::Value &c) {
                io::Value::Array z{num(".4")};
                set(profile_of(c), "z_m", io::Value{std::move(z)});
            },
            [](io::Value &c) {
                std::get<io::Value::Array>(
                        member_of(profile_of(c), "z_m").data)[1] =
                        num("nan");
            },
            [](io::Value &c) {
                set(profile_of(c), "z_m", num("1."));
            },
    };
    reject_case(golden, mutants.at(index));
    golden.done();
}

void test_pneumatic_configured_wire_requires_the_surface() {
    auto wire = fixture_wire("configured");
    erase(pneu_of(wire), "surface");
    rejects<std::invalid_argument>(
            [&] { (void)nativecfg::decode(wire); },
            "tire.pneumatic.surface");
}

void test_pneumatic_configured_wire_rejects_a_malformed_surface() {
    auto wire = fixture_wire("configured");
    erase(member_of(pneu_of(wire), "surface"), "mu_slide");
    rejects<std::invalid_argument>(
            [&] { (void)nativecfg::decode(wire); }, "mu_slide");
}

void test_pneumatic_track_wire_still_requires_the_map() {
    auto wire = fixture_wire("good_track");
    erase(tire_of(wire), "surface_map");
    rejects<std::invalid_argument>(
            [&] { (void)nativecfg::decode(wire); }, "tire.surface_map");
}

void test_pneumatic_backend_reaches_the_backend_allow_list() {
    auto wire = fixture_wire("good_track");
    set(tire_of(wire), "backend", std::string("pneumtic"));
    rejects<std::invalid_argument>(
            [&] { (void)nativecfg::decode(wire); }, "tire.backend");
    set(tire_of(wire), "backend", std::string("native_reference"));
    rejects<std::invalid_argument>(
            [&] { (void)nativecfg::decode(wire); }, "tire.backend");
}

void test_selected_native_parser_accepts_the_pneumatic_wire() {
    auto golden =
            node(kFile,
                 "test_selected_native_parser_accepts_the_pneumatic_wire");
    const auto mjb = fixture_bytes("track", "model.mjb");
    const auto cfg = nativecfg::decode(fixture_wire("track"));
    Stepper st(std::span<const std::byte>(mjb), cfg);
    const auto n = static_cast<std::size_t>(
            golden.f64_input("input.n_rays").at(0));
    const auto &names = st.tire_state_names();
    bt::require(names.size() == 2 * (4 * n + 8 + 3 * ((n + 1) / 2)),
                "state-table column count");
    text(golden, names.front(), "names.first");
    // _golden.record(np.asarray(float(len(...)))) — a 0-d <f8 entry.
    scalar(golden, static_cast<double>(names.size()), "names.count");
    bt::require(names.front() == "front.ray.000.delta_prev_m",
                "side-major wire head");
    golden.done();
}

// test_selected_native_parser_rejects_malformed_pneumatic_wires — 17
// rows; the Stepper construction boundary raises where the oracle's
// binding did.
void parser_reject_case(bt::oracle::Node &golden,
                        const std::function<void(io::Value &)> &mutate) {
    const auto mjb = fixture_bytes("track", "model.mjb");
    auto wire = fixture_wire("track");
    const auto match = input_text(golden, "input.match");
    mutate(wire);
    rejects<std::exception>(
            [&] {
                Stepper st(std::span<const std::byte>(mjb),
                           nativecfg::decode(wire));
            },
            match);
}

void test_selected_native_parser_rejects_malformed_pneumatic_wires(
        std::string_view parameters, std::size_t index) {
    auto golden =
            node(kFile,
                 "test_selected_native_parser_rejects_malformed_pneumatic_wires",
                 parameters);
    static const std::array<std::function<void(io::Value &)>, 17>
            mutants{
            [](io::Value &c) { erase(tire_of(c), "pneumatic"); },
            [](io::Value &c) {
                set(tire_of(c), "backend", std::string("compliant_2d"));
            },
            [](io::Value &c) {
                io::Value::Object material;
                material.emplace_back("radial_k_n_m", num("1.0"));
                material.emplace_back("radial_c_ns_m", num("1.0"));
                material.emplace_back("pressure_pa_gauge", num("1.0"));
                material.emplace_back("provenance",
                                      io::Value{std::string("x")});
                io::Value::Array range{num("0.0"), num("1.0")};
                material.emplace_back("valid_load_range_n",
                                      io::Value{std::move(range)});
                set(member_of(tire_of(c), "front"), "material",
                    std::move(material));
            },
            [](io::Value &c) {
                set(pneu_of(c), "misspelled", num("1"));
            },
            [](io::Value &c) {
                set(pneu_of(c), "tier", std::string("ultra"));
            },
            [](io::Value &c) { set(pneu_of(c), "n_rays", num("128")); },
            [](io::Value &c) {
                set(pneu_of(c), "discretised_brush", std::string("yes"));
            },
            [](io::Value &c) {
                set(pneu_of(c), "timestep_s", num(".001"));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "front"), "pressure_bar",
                    num("-1."));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "front"), "rim_radius_mm",
                    num("400."));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "rear"), "area_factor",
                    num("1.5"));
            },
            [](io::Value &c) {
                set(member_of(pneu_of(c), "rear"), "loss_factor",
                    num("1."));
            },
            [](io::Value &c) {
                set(profile_of(c), "dx_m", num("0."));
            },
            [](io::Value &c) {
                set(profile_of(c), "z_m", io::Value{io::Value::Array{}});
            },
            [](io::Value &c) {
                io::Value::Array z{num(".4")};
                set(profile_of(c), "z_m", io::Value{std::move(z)});
            },
            [](io::Value &c) {
                io::Value::Array z{num(".4"), num("nan")};
                set(profile_of(c), "z_m", io::Value{std::move(z)});
            },
            [](io::Value &c) {
                set(tire_of(c), "surface_mode", std::string("configured"));
            },
    };
    parser_reject_case(golden, mutants.at(index));
    golden.done();
}

// ---------------------------------------------------------------------
// pytest node ids — the oracle's parametrize generates
// <lambda>-<match><ordinal> for repeated match strings.
// ---------------------------------------------------------------------

constexpr std::array kRejectIds{
        "<lambda>-tire.pneumatic0",
        "<lambda>-tire.pneumatic1",
        "<lambda>-tire.front.material",
        "<lambda>-tire.pneumatic.misspelled",
        "<lambda>-tire.pneumatic.profile0",
        "<lambda>-tire.pneumatic.profile1",
        "<lambda>-tire.pneumatic.profile2",
        "<lambda>-tire.pneumatic.tier",
        "<lambda>-tire.pneumatic.n_rays0",
        "<lambda>-tire.pneumatic.n_rays1",
        "<lambda>-tire.pneumatic.discretised_brush",
        "<lambda>-tire.pneumatic.timestep_s",
        "<lambda>-tire.pneumatic.front0",
        "<lambda>-tire.pneumatic.front.pressure_bar",
        "<lambda>-tire.pneumatic.front.rim_radius_mm",
        "<lambda>-tire.pneumatic.front1",
        "<lambda>-tire.pneumatic.front.loss_factor",
        "<lambda>-tire.pneumatic.front2",
        "<lambda>-profile.dx_m",
        "<lambda>-profile.z_m0",
        "<lambda>-profile.z_m1",
        "<lambda>-profile.z_m2",
        "<lambda>-profile.z_m3",
};

constexpr std::array kParserRejectIds{
        "<lambda>-tire.pneumatic0",
        "<lambda>-tire.pneumatic1",
        "<lambda>-tire.front.material",
        "<lambda>-tire.pneumatic.misspelled",
        "<lambda>-tire.pneumatic.tier",
        "<lambda>-tire.pneumatic.n_rays",
        "<lambda>-tire.pneumatic.discretised_brush",
        "<lambda>-tire.pneumatic.timestep_s",
        "<lambda>-tire.pneumatic.front.pressure_bar",
        "<lambda>-tire.pneumatic.front.rim_radius_mm",
        "<lambda>-tire.pneumatic.rear",
        "<lambda>-tire.pneumatic.rear.loss_factor",
        "<lambda>-profile.dx_m",
        "<lambda>-profile.z_m0",
        "<lambda>-profile.z_m1",
        "<lambda>-profile.z_m2",
        "<lambda>-tire.pneumatic.surface",
};

} // namespace

// cppcheck-suppress throwInEntryPoint
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_pneumatic_projection_carries_the_full_section();
        test_pneumatic_profile_matches_the_appliers_road_bitwise();
        test_pneumatic_track_mode_keeps_the_surface_map();
        test_pneumatic_wire_round_trips_through_the_schema();
        test_configured_surface_projects_the_resolved_spec();
        test_configured_surface_projects_through_a_real_sim();
        test_named_surface_overrides_the_track_map_on_the_wire();
        test_detailed_tier_projects_its_resolved_parameters();
        test_pneumatic_section_requires_the_road_profile();
        test_projection_rejects_a_non_pneumatic_section();
        for (std::size_t i = 0; i < std::size(kRejectIds); ++i)
            test_pneumatic_wire_rejection_cases(kRejectIds.at(i), i);
        test_pneumatic_configured_wire_requires_the_surface();
        test_pneumatic_configured_wire_rejects_a_malformed_surface();
        test_pneumatic_track_wire_still_requires_the_map();
        test_pneumatic_backend_reaches_the_backend_allow_list();
        test_selected_native_parser_accepts_the_pneumatic_wire();
        for (std::size_t i = 0; i < std::size(kParserRejectIds); ++i)
            test_selected_native_parser_rejects_malformed_pneumatic_wires(
                    kParserRejectIds.at(i), i);
    });
}
NATIVE_DIAG_POP
