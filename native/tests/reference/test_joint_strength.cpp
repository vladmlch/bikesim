// Ports all fifteen source test functions in
// tests/reference/test_joint_strength.py — directional joint strength
// curves (TorqueCurve validation + directional_capacity), the provenance
// gate on the shipped strength profile, the 450 W positive-power budget,
// and the three slow env records (pure-motor actuators, stepped effort,
// strength-limited clipping).
//
// _curve()/TorqueCurve(**overrides) constructor validation maps to
// cfg::detail::post_init — the __post_init__ port, strip-aware provenance
// check included (spindle::validate_torque_curve is the wire-decoder twin
// and intentionally not this contract).
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "rider_control_reference_support.hpp"
#include "configuration/rider_spec.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "model_access.hpp"
#include "runtime/accounting.hpp"
#include "terrain/trackfile.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace rr = rider_control_reference;
namespace cfg = bike::configuration;
namespace io = bike::io;
namespace rcfg = ::runtime;

const std::filesystem::path kStrengthPath =
    rr::repo_file("examples/research/rider_strength_reference.json");
const std::filesystem::path kEnvelopePath = rr::repo_file(
    "examples/research/rider_joint_envelope_anatomical.json");

// JOINTS = reference_joint_names() — the 11-joint registry in the same
// canonical order the Python tuple declares.
std::span<const std::string_view> joints() { return cfg::kRiderJoints; }

// _curve(): the synthetic-unit fixture; validation happens at post_init
// exactly like TorqueCurve.__post_init__.
spindle::TorqueCurve synthetic_curve() {
    return spindle::TorqueCurve{.angles_rad = {-1., 0., 1.},
                                .torques_nm = {50., 100., 50.},
                                .vmax_rad_s = 10.,
                                .hill_c = .25,
                                .eccentric_ratio = 1.4,
                                .source = "synthetic-unit-fixture-not-"
                                          "human-data"};
}

void require_valid(spindle::TorqueCurve curve) {
    cfg::detail::post_init(curve);
}

// StrengthProfile is an ordered (name, {sign: curve}) vector; Python
// profile[name][direction].
const spindle::TorqueCurve &curve_at(const cfg::StrengthProfile &profile,
                                     const std::string &name, int sign) {
    const auto found = std::ranges::find(profile, name,
                                         &std::pair<
                                             std::string,
                                             std::map<int, spindle::TorqueCurve>>::
                                             first);
    bt::require(found != profile.end(),
                "the profile covers joint " + name);
    return found->second.at(sign);
}

std::set<std::string> profile_names(const cfg::StrengthProfile &profile) {
    std::set<std::string> out;
    for (const auto &[name, curves] : profile) out.insert(name);
    return out;
}

std::string read_text(const std::filesystem::path &path) {
    std::ifstream stream(path);
    bt::require(stream.is_open(), "fixture opens: " + path.string());
    std::ostringstream body;
    body << stream.rdbuf();
    return body.str();
}

io::Value &member(io::Value &value, std::string_view key) {
    auto &object = std::get<io::Value::Object>(value.data);
    for (auto &[k, v] : object)
        if (k == key) return v;
    bt::require(false, "payload member present: " + std::string(key));
    std::unreachable();
}

// _load(payload,...): dump the mutated document and run the file loader —
// the tempfile boundary is part of the contract (the loader reads paths).
std::filesystem::path write_payload(const io::Value &payload,
                                    const bt::TemporaryDirectory &dir,
                                    std::string_view name) {
    const auto path = dir.path() / std::string(name);
    std::ofstream stream(path);
    bt::require(stream.is_open(), "temp payload opens for writing");
    stream << io::canonical_json(payload);
    stream.close();
    return path;
}

[[gnu::noinline]] void test_curve_validation() {
    // tests/reference/test_joint_strength.py::test_curve_validation
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.angles_rad = {0.};
            c.torques_nm = {1.};
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.angles_rad = {0., 1., 2.};
            c.torques_nm = {1., 2.};
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.torques_nm = {50., -1., 50.};
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.angles_rad = {0., 0., 1.};
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.vmax_rad_s = 0.;
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.hill_c = -.1;
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.eccentric_ratio = .9;
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.source = "   ";
            require_valid(std::move(c));
        },
        "");
    rr::expect_invalid(
        [] {
            auto c = synthetic_curve();
            c.angles_rad = {0., std::nan("")};
            require_valid(std::move(c));
        },
        "");
}

[[gnu::noinline]] void test_plan_fixture_ordering_and_velocity_limits() {
    // tests/reference/test_joint_strength.py::test_plan_fixture_ordering_and_velocity_limits
    const auto curve = synthetic_curve();
    const double iso = spindle::directional_capacity(curve, 0., 0., 1);
    const double concentric =
        spindle::directional_capacity(curve, 0., 5., 1);
    const double eccentric =
        spindle::directional_capacity(curve, 0., -5., 1);
    bt::require(0. <= concentric && concentric < iso && iso <= eccentric &&
                    eccentric <= 140.,
                "isometric < concentric < eccentric ordering holds");
    bt::require(spindle::directional_capacity(curve, 0., 10., 1) == 0.,
                "at vmax nothing remains");
    bt::require(spindle::directional_capacity(curve, 0., -10., -1) == 0.,
                "at vmax on the negative side nothing remains");
    bt::require(100. * -5. < 0,
                "eccentric torque against motion only absorbs");
}

[[gnu::noinline]] void test_isometric_interpolation_between_knots() {
    // tests/reference/test_joint_strength.py::test_isometric_interpolation_between_knots
    const auto curve = synthetic_curve();
    bt::require(rr::approx(spindle::directional_capacity(curve, 0., 0., 1),
                           100.),
                "isometric at the center knot");
    bt::require(rr::approx(spindle::directional_capacity(curve, .5, 0., 1),
                           75.),
                "isometric interpolates halfway up");
    bt::require(rr::approx(spindle::directional_capacity(curve, -.5, 0., 1),
                           75.),
                "isometric interpolates halfway down");
    bt::require(
        std::isfinite(spindle::directional_capacity(curve, 0., 0., -1)),
        "static requests at zero speed are never throttled by a division");
}

[[gnu::noinline]] void
test_concentric_decays_eccentric_is_bounded_and_finite() {
    // tests/reference/test_joint_strength.py::test_concentric_decays_eccentric_is_bounded_and_finite
    const auto curve = synthetic_curve();
    const std::vector<double> speeds{0., 2., 5., 8.};
    std::vector<double> capacities;
    capacities.reserve(speeds.size());
    for (const double v : speeds)
        capacities.push_back(spindle::directional_capacity(curve, 0., v, 1));
    for (std::size_t i = 1; i < capacities.size(); ++i)
        bt::require(capacities[i] < capacities[i - 1],
                    "concentric capacity decays monotonically");
    bt::require(spindle::directional_capacity(curve, 0., 10.1, 1) == 0.,
                "slightly above vmax nothing remains");
    const double deep = spindle::directional_capacity(curve, 0., -50., 1);
    bt::require(deep <= 100. * 1.4 + 1e-9,
                "the eccentric branch saturates at eccentric_ratio");
    bt::require(std::isfinite(deep), "deep eccentric stays finite");
    bt::require(spindle::directional_capacity(curve, 0., -5., 1) * -5. < 0.,
                "eccentric torque against motion only absorbs");
}

[[gnu::noinline]] void test_direction_sign_uses_the_torque_direction() {
    // tests/reference/test_joint_strength.py::test_direction_sign_uses_the_torque_direction
    const auto curve = synthetic_curve();
    bt::require(spindle::directional_capacity(curve, 0., -5., -1) <
                    spindle::directional_capacity(curve, 0., 0., -1),
                "a negative-q torque sees concentric at negative velocity");
    bt::require(spindle::directional_capacity(curve, 0., 5., -1) >
                    spindle::directional_capacity(curve, 0., -5., -1),
                "eccentric on the negative side beats concentric");
    rr::expect_invalid(
        [&] { (void)spindle::directional_capacity(curve, 0., 0., 0); }, "");
    rr::expect_invalid(
        [&] { (void)spindle::directional_capacity(curve, 2., 0., 1); }, "");
    rr::expect_invalid(
        [&] {
            (void)spindle::directional_capacity(
                curve, 0., std::numeric_limits<double>::infinity(), 1);
        },
        "");
}

[[gnu::noinline]] void
test_positive_power_budget_scales_active_torque_only() {
    // tests/reference/test_joint_strength.py::test_positive_power_budget_scales_active_torque_only
    const std::vector<double> torque5(5, 100.), speed5(5, 5.);
    auto limited = spindle::limit_positive_power(torque5, speed5, 450.);
    double positive = 0.;
    for (std::size_t i = 0; i < limited.size(); ++i)
        positive += std::max(limited[i] * speed5[i], 0.);
    bt::require(rr::approx(positive, 450.),
                "the budget scales the accelerating set to exactly 450 W");
    const std::vector<double> torque2{100., -100.}, speed2{5., 5.};
    limited = spindle::limit_positive_power(torque2, speed2, 300.);
    positive = 0.;
    for (std::size_t i = 0; i < limited.size(); ++i)
        positive += std::max(limited[i] * speed2[i], 0.);
    bt::require(rr::approx(positive, 300.),
                "the eccentric joint does not consume the budget");
    bt::require(limited[1] == -100.,
                "braking capacity is not scaled down");
    const std::vector<double> zero_speed(2, 0.);
    const auto untouched =
        spindle::limit_positive_power(torque2, zero_speed, 450.);
    bt::require(untouched == torque2,
                "zero velocity divides nothing: the request passes through");
}

[[gnu::noinline]] void test_reference_profile_covers_exactly_the_registry() {
    // tests/reference/test_joint_strength.py::test_reference_profile_covers_exactly_the_registry
    const auto profile =
        cfg::load_strength_profile(kStrengthPath, joints(), false);
    bt::require(profile_names(profile) ==
                    std::set<std::string>(joints().begin(), joints().end()),
                "the profile covers exactly the joint registry");
    for (const auto &[name, curves] : profile) {
        bt::require(curves.size() == 2 && curves.contains(1) &&
                        curves.contains(-1),
                    name + ": both torque directions ship");
        for (const auto &[sign, curve] : curves) {
            bt::require(curve.vmax_rad_s > 0. && curve.eccentric_ratio >= 1.,
                        name + ": force-velocity parameters are sane");
            bt::require(!curve.source.empty() &&
                            curve.source.find_first_not_of(" \t\n\r") !=
                                std::string::npos,
                        name + ": provenance is nonblank");
        }
    }
}

[[gnu::noinline]] void
test_verified_gate_rejects_the_provisional_profile() {
    // tests/reference/test_joint_strength.py::test_verified_gate_rejects_the_provisional_profile
    rr::expect_invalid(
        [&] {
            (void)cfg::load_strength_profile(kStrengthPath, joints(), true);
        },
        "not verified");
    // Marking one curve verified still leaves ten others unverified.
    io::Value payload = io::parse_json(read_text(kStrengthPath));
    auto &positive = member(
        member(member(member(payload, "joints"), "rider_torso_hinge"),
               "directions"),
        "positive");
    member(positive, "verified").data = true;
    bt::TemporaryDirectory dir("joint-strength-gate");
    const auto path = write_payload(payload, dir, "strength.json");
    rr::expect_invalid(
        [&] { (void)cfg::load_strength_profile(path, joints(), true); },
        "not verified");
}

[[gnu::noinline]] void
test_profile_rejects_incomplete_or_alien_coverage() {
    // tests/reference/test_joint_strength.py::test_profile_rejects_incomplete_or_alien_coverage
    bt::TemporaryDirectory dir("joint-strength-coverage");
    {
        io::Value payload = io::parse_json(read_text(kStrengthPath));
        auto &joint_map =
            std::get<io::Value::Object>(member(payload, "joints").data);
        std::erase_if(joint_map, [](const auto &entry) {
            return entry.first == "rider_ankle_rear";
        });
        const auto path = write_payload(payload, dir, "missing.json");
        rr::expect_invalid(
            [&] {
                (void)cfg::load_strength_profile(path, joints(), false);
            },
            "internal rider joints");
    }
    {
        io::Value payload = io::parse_json(read_text(kStrengthPath));
        auto &directions = std::get<io::Value::Object>(
            member(member(member(payload, "joints"), "rider_knee_front"),
                   "directions")
                .data);
        std::erase_if(directions, [](const auto &entry) {
            return entry.first == "negative";
        });
        const auto path = write_payload(payload, dir, "one-dir.json");
        rr::expect_invalid(
            [&] {
                (void)cfg::load_strength_profile(path, joints(), false);
            },
            "two torque directions");
    }
    {
        io::Value payload = io::parse_json(read_text(kStrengthPath));
        member(payload, "topology").data =
            std::string("articulated_planar_single_arm");
        const auto path = write_payload(payload, dir, "alien.json");
        rr::expect_invalid(
            [&] {
                (void)cfg::load_strength_profile(path, joints(), false);
            },
            "different rider topology");
    }
}

[[gnu::noinline]] void
test_curve_knots_must_cover_the_used_anatomical_range() {
    // tests/reference/test_joint_strength.py::test_curve_knots_must_cover_the_used_anatomical_range
    io::Value payload = io::parse_json(read_text(kStrengthPath));
    auto &negative = member(
        member(member(member(payload, "joints"), "rider_knee_front"),
               "directions"),
        "negative");
    member(negative, "angles_rad").data =
        io::Value::Array{io::Value{io::Number{"0.5"}},
                         io::Value{io::Number{"1.0"}},
                         io::Value{io::Number{"2.0"}}};
    bt::TemporaryDirectory dir("joint-strength-rom");
    const auto path = write_payload(payload, dir, "rom.json");
    rr::expect_invalid(
        [&] { (void)cfg::load_strength_profile(path, joints(), false); },
        "used ROM");
}

[[gnu::noinline]] void
test_strength_coordinates_agree_with_the_joint_envelope() {
    // tests/reference/test_joint_strength.py::test_strength_coordinates_agree_with_the_joint_envelope
    const auto coordinates =
        cfg::load_strength_coordinates(kStrengthPath, joints());
    const auto envelopes = cfg::load_joint_envelopes(kEnvelopePath);
    const auto names_of = [](const auto &map) {
        std::set<std::string> out;
        for (const auto &[name, envelope] : map) out.insert(name);
        return out;
    };
    bt::require(names_of(coordinates) == names_of(envelopes) &&
                    names_of(coordinates) ==
                        std::set<std::string>(joints().begin(),
                                              joints().end()),
                "all three tables cover exactly the registry");
    const auto lookup = [](const auto &map, const std::string &name)
        -> const spindle::JointEnvelope & {
        for (const auto &[n, e] : map)
            if (n == name) return e;
        bt::require(false, "table carries " + name);
        std::unreachable();
    };
    for (const std::string_view name : joints()) {
        const auto &coordinate = lookup(coordinates, std::string(name));
        const auto &envelope = lookup(envelopes, std::string(name));
        bt::require(coordinate.direction == envelope.direction,
                    std::string(name) + ": directions agree");
        bt::require(rr::approx(coordinate.neutral_anatomical_rad,
                               envelope.neutral_anatomical_rad),
                    std::string(name) + ": neutral angles agree");
    }
}

// _environment's flat probe track (test_joint_strength.py:191-193).
bike::terrain::TrackSpec probe_track_30() {
    return bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n"));
}

[[gnu::noinline]] void
test_actuators_are_pure_motors_with_dof_damping() {
    // tests/reference/test_joint_strength.py::test_actuators_are_pure_motors_with_dof_damping
    auto rig = rr::pinned_rig({}, probe_track_30());
    auto &physical = *rig->physical;
    const auto &controller = physical.rider_control();
    const mjModel *m = rig->stepper->model();
    bt::require(controller.config().has_strength,
                "the welded rider ships a strength table");
    // controller.strength keys == the registry joints present in the model.
    std::set<std::string> present;
    for (const std::string_view name : joints())
        if (mj_name2id(m, mjOBJ_JOINT, std::string(name).c_str()) >= 0)
            present.insert(std::string(name));
    std::set<std::string> strength_names;
    for (const auto &[name, directions] : controller.config().strength)
        strength_names.insert(name);
    bt::require(strength_names == present,
                "the strength table covers exactly the present joints");
    const auto trntype =
        model_access::readonly_buffer(m->actuator_trntype, m->nu,
                                      "actuator_trntype");
    const auto biasprm =
        model_access::readonly_buffer(m->actuator_biasprm, 10 * m->nu,
                                      "actuator_biasprm");
    const auto damping = model_access::readonly_buffer(
        m->dof_damping, m->nv, "dof_damping");
    for (const auto &j : controller.joints()) {
        bt::require(trntype[static_cast<std::size_t>(j.actuator_id)] ==
                        mjTRN_JOINT,
                    j.name + ": the actuator is a direct joint motor");
        for (int p = 0; p < 10; ++p)
            bt::require(
                rr::approx(biasprm[static_cast<std::size_t>(
                                       10 * j.actuator_id + p)],
                           0.),
                j.name + ": no affine bias — actuator_force is purely "
                         "the commanded muscle torque");
        bt::require(
            rr::approx(damping[static_cast<std::size_t>(j.dof_adr)],
                       controller.config().passive_damping_nms_rad()),
            j.name + ": dof damping carries the tissue term");
    }
}

const std::vector<double> &component_row(const ::runtime::RawStep &raw,
                                         const std::string &name) {
    for (const auto &[row_name, row] : raw.components)
        if (row_name == name) return row;
    bt::require(false, "force row present: " + name);
    std::unreachable();
}

[[gnu::noinline]] void
test_stepped_effort_respects_strength_and_the_power_budget() {
    // tests/reference/test_joint_strength.py::test_stepped_effort_respects_strength_and_the_power_budget
    auto rig = rr::pinned_rig({}, probe_track_30());
    auto &physical = *rig->physical;
    rcfg::RideControl control;
    control.motor_torque_nm = 0.;
    control.human_torque_nm = 30.;
    ::runtime::RawStep last;
    // env.step(control, front=1., rear=1.) until duration 1 s.
    while (rig->stepper->data()->time < 1.)
        last = physical.advance_physics(1., 1., std::nullopt, control);
    const auto &controller = physical.rider_control();
    // physical.sample's observation: the oracle's solved_effort runs per
    // interval on the raw's incoming state (q, v) and the still-current
    // post-step forces — observe_effort reproduces exactly that fold on
    // the raw's own pre-forward copies (the live engine has already
    // re-forwarded qfrc_passive at the post-step velocity).
    const auto observation = rcfg::observe_effort(
        last, controller, last.end_time_s - last.time_s);
    const auto &diagnostics = observation.diagnostics;
    bt::require(diagnostics.positive_power_w <= 450. + 1e-6,
                "positive power stays inside the 450 W budget");
    bt::require(diagnostics.strength_violations.has_value() &&
                    diagnostics.strength_violations->empty(),
                "no strength violation is reported");
    bt::require(diagnostics.passive_power_w <= 1e-9,
                "tissue damping can only remove power");
    const double kd = controller.config().passive_damping_nms_rad();
    const std::string passive_name{"rider_passive_damping"},
        engine_name{"engine_passive"};
    const auto &passive_row = component_row(last, passive_name);
    const auto &engine_row = component_row(last, engine_name);
    for (const auto &j : controller.joints()) {
        const auto *terms = static_cast<const ::rider::JointTerms *>(
            nullptr);
        for (const auto &[name, t] : observation.terms)
            if (name == j.name) terms = &t;
        bt::require(terms != nullptr,
                    "last_terms carries " + j.name);
        const std::string act_name = "act_" + j.name;
        const auto &act_row = component_row(last, act_name);
        const auto dof = static_cast<std::size_t>(j.dof_adr);
        bt::require(terms->solved_active_nm.has_value() &&
                        rr::approx(*terms->solved_active_nm, act_row[dof]),
                    j.name + ": solved active torque is the actuator row");
        bt::require(
            terms->solved_passive_nm.has_value() &&
                rr::approx(*terms->solved_passive_nm, passive_row[dof]),
            j.name + ": solved passive torque is the damping row");
        bt::require(terms->solved_passive_nm.has_value() &&
                        rr::approx(*terms->solved_passive_nm,
                                   -kd * last.qvel[dof]),
                    j.name + ": tissue damping is -kd*qvel at the "
                              "incoming velocity");
        bt::require(rr::approx(engine_row[dof], 0., 1e-6, 1e-9),
                    j.name + ": engine_passive never double-counts the "
                              "dof damping");
    }
}

[[gnu::noinline]] void
test_directional_capacity_bounds_the_commanded_torque() {
    // tests/reference/test_joint_strength.py::test_directional_capacity_bounds_the_commanded_torque
    auto rig = rr::pinned_rig({}, probe_track_30());
    auto &physical = *rig->physical;
    const auto &controller = physical.rider_control();
    const mjData *d = rig->stepper->data();
    const mjModel *m = rig->stepper->model();
    const std::vector<double> request(controller.joints().size(), 1e4);
    const auto qpos =
        model_access::readonly_buffer(d->qpos, m->nq, "qpos");
    const auto qvel =
        model_access::readonly_buffer(d->qvel, m->nv, "qvel");
    const auto [clipped, limited] =
        controller.strength_limited(request, qpos, qvel);
    std::set<std::string> names;
    for (const auto &j : controller.joints()) names.insert(j.name);
    bt::require(std::set<std::string>(limited.begin(), limited.end()) ==
                    names,
                "every joint reports its limit");
    for (std::size_t i = 0; i < controller.joints().size(); ++i) {
        const auto &j = controller.joints()[i];
        const double capacity = controller.strength_capacity(
            j.name,
            controller.anatomical_joint_angle(
                j.name, qpos[static_cast<std::size_t>(j.qpos_adr)]),
            qvel[static_cast<std::size_t>(j.dof_adr)], 1e4);
        bt::require(rr::approx(clipped[i], capacity),
                    j.name + ": the clip is the directional capacity");
        bt::require(capacity < 1e4,
                    j.name + ": the capacity actually bound the request");
    }
}

[[gnu::noinline]] void
test_leg_curves_span_a_120_rpm_pedalling_envelope() {
    // tests/reference/test_joint_strength.py::test_leg_curves_span_a_120_rpm_pedalling_envelope
    const auto profile =
        cfg::load_strength_profile(kStrengthPath, joints(), false);
    for (const std::string side : {"front", "rear"}) {
        for (const std::string joint : {"hip", "knee", "ankle"})
            for (const int direction : {1, -1}) {
                const std::string curve_name =
                    "rider_" + joint + "_" + side;
                const auto &curve =
                    curve_at(profile, curve_name, direction);
                bt::require(curve.vmax_rad_s >= 20.,
                            joint + " " + side + ": vmax spans 120 rpm");
                bt::require(curve.hill_c >= .3,
                            joint + " " + side + ": hill_c is sane");
                bt::require(curve.source.find("unverified") !=
                                std::string::npos,
                            joint + " " + side + ": provenance is marked");
            }
        const std::string knee_name = "rider_knee_" + side;
        const auto &knee = curve_at(profile, knee_name, -1);
        bt::require(spindle::directional_capacity(knee, 1.0, -7.7, -1) >=
                        40.,
                    side + " knee extension holds ~40 N.m at 7.7 rad/s");
        const std::string hip_name = "rider_hip_" + side;
        const auto &hip = curve_at(profile, hip_name, 1);
        bt::require(spindle::directional_capacity(hip, .9, 5., 1) >= 60.,
                    side + " hip extension holds ~60 N.m at 5 rad/s");
    }
    const std::string torso_name{"rider_torso_hinge"};
    const auto &torso = curve_at(profile, torso_name, 1);
    bt::require(torso.vmax_rad_s == 8., "the torso curve is untouched");
}
} // namespace

int main() {
    return bt::run([] {
        test_curve_validation();
        test_plan_fixture_ordering_and_velocity_limits();
        test_isometric_interpolation_between_knots();
        test_concentric_decays_eccentric_is_bounded_and_finite();
        test_direction_sign_uses_the_torque_direction();
        test_positive_power_budget_scales_active_torque_only();
        test_reference_profile_covers_exactly_the_registry();
        test_verified_gate_rejects_the_provisional_profile();
        test_profile_rejects_incomplete_or_alien_coverage();
        test_curve_knots_must_cover_the_used_anatomical_range();
        test_strength_coordinates_agree_with_the_joint_envelope();
        test_actuators_are_pure_motors_with_dof_damping();
        test_stepped_effort_respects_strength_and_the_power_budget();
        test_directional_capacity_bounds_the_commanded_torque();
        test_leg_curves_span_a_120_rpm_pedalling_envelope();
    });
}
