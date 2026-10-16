// test_t14.cpp — T14 geometry + Horst-link kinematics migration
// regressions. Covers the owned declarations behind the T14 boundary:
// the three specs.py records with declared defaults, hardpoints.py's
// hardpoint/trail/frame-angle calculators, validation.py's domain
// validator (invalid-state messages verbatim), cockpit.py's
// metre-domain saddle/seatpost/grip/pedal placement + inverse +
// mass split, linkage_math.py's brentq/circle primitives (endpoint,
// reversed, unbracketed, NaN, tangent, concentric and clamped-edge
// branches), solver.py's HorstLinkageSolver construction/state/
// trajectory/inverse solves incl. residual link errors, and
// curves.py's instant-center curve — every comparison bitwise
// against the oracle fixtures declared in the manifest (comparison B),
// with fixture docs driving the replay inputs.
#include "support/migration_test.hpp"

#include "geometry/specs.hpp"
#include "io/json.hpp"
#include "kinematics/linkage_math.hpp"
#include "kinematics/solver.hpp"

#include <bit>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

namespace geo = bike::geometry;
namespace kin = bike::kinematics;
namespace io = bike::io;
using io::Value;

// ---- JSON helpers (same idioms as test_t12) ---------------------------------

const Value::Object &object_of(const Value &value,
                               std::string_view field) {
    return io::as_object(value, field);
}
const Value &member(const Value &object, std::string_view key,
                    std::string_view path) {
    const Value *found = io::find_member(object_of(object, path), key);
    bt::require(found != nullptr,
                "missing member " + std::string(path) + "." +
                        std::string(key));
    return *found;
}
const Value *member_opt(const Value &object, std::string_view key,
                        std::string_view path) {
    return io::find_member(object_of(object, path), key);
}
double number_of(const Value &value, std::string_view field) {
    const std::string lexeme = io::as_number(value, field).lexeme;
    bool ok = false;
    const double result = io::detail::float_from_lexeme(lexeme, ok);
    bt::require(ok, "number lexeme did not decode: " + lexeme);
    return result;
}
bool bool_of(const Value &value, std::string_view field) {
    const auto *flag = std::get_if<bool>(&value.data);
    bt::require(flag != nullptr, std::string(field) + " must be a bool");
    return *flag;
}
const std::string &string_of(const Value &value,
                             std::string_view field) {
    const auto *text = std::get_if<std::string>(&value.data);
    bt::require(text != nullptr,
                std::string(field) + " must be a string");
    return *text;
}
const Value::Array &array_of(const Value &value,
                             std::string_view field) {
    const auto *arr = std::get_if<Value::Array>(&value.data);
    bt::require(arr != nullptr,
                std::string(field) + " must be an array");
    return *arr;
}
std::vector<double> doubles_of(const Value &value,
                               std::string_view field) {
    std::vector<double> out;
    for (const Value &v: array_of(value, field))
        out.push_back(number_of(v, field));
    return out;
}

// A manifest-validated JSON doc: the entry's declared files verify
// (bytes + SHA-256) before the doc text is accepted — the fail-closed
// acquisition path.
Value doc_of(std::string_view name) {
    bt::manifest_verify_files(bt::manifest_entry(name));
    return io::parse_json(
            bt::text(bt::fixture(std::string(name) + ".json")));
}

// ---- $array reference helpers ------------------------------------------------

// Validate the declared dtype/order/shape of a $array reference and
// return the fixture-relative path.
std::string array_path(const Value &ref, std::string_view label) {
    bt::require(string_of(member(ref, "dtype", label), label) == "<f8",
                std::string(label) + ": dtype must be <f8");
    bt::require(string_of(member(ref, "order", label), label) == "C",
                std::string(label) + ": order must be C");
    return string_of(member(ref, "$array", label), label);
}
std::size_t shape_count(const Value &ref, std::string_view label) {
    std::size_t count = 1;
    for (const Value &v: array_of(member(ref, "shape", label), label))
        count *= static_cast<std::size_t>(number_of(v, label));
    return count;
}
// Bitwise binary64 comparison of the produced values against the
// referenced golden .f64 (signed zeros and NaN payloads significant).
void expect_bits(const Value &ref, std::span<const double> actual,
                 std::string_view label) {
    bt::require(shape_count(ref, label) == actual.size(),
                std::string(label) + ": declared shape does not match");
    bt::golden_f64(array_path(ref, label), actual);
}
void expect_bits(const Value &ref, const geo::Vec3 &actual,
                 std::string_view label) {
    expect_bits(ref, std::span<const double>(actual), label);
}
// Decode a JSON repr number back to its source double (exact
// round-trip through the recorded repr).
double scalar_of(const Value &v, std::string_view label) {
    return number_of(v, label);
}

void expect_invalid_msg(std::string_view expected,
                        const std::function<void()> &action,
                        std::string_view label) {
    try {
        action();
    } catch (const std::invalid_argument &error) {
        bt::require(expected == error.what(),
                    std::string(label) + ": invalid_argument message "
                            "mismatch: '" +
                            std::string(error.what()) + "'");
        return;
    }
    bt::require(false,
                std::string(label) + ": expected std::invalid_argument");
}

// ---- geometry/specs.cpp -----------------------------------------------------

void test_specs_defaults() {
    const geo::BikeSpecs s;
    // Frame geometry defaults (specs.py declared order).
    bt::require(s.reach == 480.0 && s.stack == 646.0,
                "BikeSpecs reach/stack defaults");
    bt::require(s.head_angle_deg == 64.0 &&
                    s.effective_seat_angle_deg == 77.0 &&
                    s.bb_drop == 22.5 && s.wheelbase == 1280.55,
                "BikeSpecs angle/drop/wheelbase defaults");
    bt::require(s.headtube_length == 120.0 &&
                    s.front_wheel_radius == 372.0 &&
                    s.rear_wheel_radius == 352.0 &&
                    s.wheel_radius == 372.0,
                "BikeSpecs tube/wheel defaults");
    bt::require(s.crank_length == 165.0, "crank_length default");
    // Suspension hardware defaults.
    bt::require(s.fork_travel == 180.0 && s.fork_offset == 44.0 &&
                    s.rear_wheel_travel == 180.0 &&
                    s.shock_eye_to_eye == 205.0 &&
                    s.shock_stroke == 65.0,
                "BikeSpecs travel/offset/shock defaults");
    bt::require(s.fork_stiffness == 6000.0 &&
                    s.fork_damping == 450.0 &&
                    s.shock_stiffness == 114600.0 &&
                    s.shock_damping == 1200.0,
                "BikeSpecs rate defaults");
    // Damper/spring settings defaults.
    bt::require(s.fork_hsc == 2 && s.fork_lsc == 7 &&
                    s.fork_rebound == 9 && s.fork_initial_psi == 85.2 &&
                    s.fork_air_tokens == 2,
                "BikeSpecs fork damper defaults");
    bt::require(s.shock_hsc == 2 && s.shock_lsc == 7 &&
                    s.shock_rebound == 7 && s.shock_hbo == 2 &&
                    !s.shock_lockout,
                "BikeSpecs shock damper defaults");

    // The geometry/suspension property views carry the same values in
    // the nested record order.
    const geo::FrameGeometrySpecs g = s.geometry();
    bt::require(g.reach == s.reach && g.stack == s.stack &&
                    g.head_angle_deg == s.head_angle_deg &&
                    g.effective_seat_angle_deg ==
                            s.effective_seat_angle_deg &&
                    g.bb_drop == s.bb_drop &&
                    g.wheelbase == s.wheelbase &&
                    g.headtube_length == s.headtube_length &&
                    g.front_wheel_radius == s.front_wheel_radius &&
                    g.rear_wheel_radius == s.rear_wheel_radius &&
                    g.wheel_radius == s.wheel_radius,
                "geometry() property record");
    const geo::SuspensionHardwareSpecs h = s.suspension();
    bt::require(h.fork_travel == s.fork_travel &&
                    h.fork_offset == s.fork_offset &&
                    h.rear_wheel_travel == s.rear_wheel_travel &&
                    h.shock_eye_to_eye == s.shock_eye_to_eye &&
                    h.shock_stroke == s.shock_stroke &&
                    h.fork_stiffness == s.fork_stiffness &&
                    h.fork_damping == s.fork_damping &&
                    h.shock_stiffness == s.shock_stiffness &&
                    h.shock_damping == s.shock_damping,
                "suspension() property record");
    // Nested records keep their own declared defaults too.
    bt::require(geo::FrameGeometrySpecs{}.bb_drop == 22.5 &&
                    geo::SuspensionHardwareSpecs{}.shock_stroke == 65.0,
                "nested record standalone defaults");
}

// ---- geometry/hardpoints -----------------------------------------------------

void test_hardpoints() {
    const Value doc = doc_of("geometry/hardpoints");
    const geo::BikeSpecs specs;

    const geo::HeadtubePoints ht = geo::compute_headtube_points(specs);
    expect_bits(member(doc, "p11_top", "doc"), ht.top, "p11_top");
    expect_bits(member(doc, "p_ht_bot", "doc"), ht.bottom,
                "p_ht_bot");

    const geo::Vec3 fa = geo::compute_front_axle(specs);
    expect_bits(member(doc, "front_axle", "doc"), fa, "front_axle");

    const double gz = geo::compute_ground_z(specs);
    const std::array<double, 1> gz_arr{gz};
    expect_bits(member(doc, "ground_z", "doc"),
                std::span<const double>(gz_arr), "ground_z");

    expect_bits(member(doc, "rear_axle", "doc"),
                geo::compute_rear_axle(specs), "rear_axle");
    expect_bits(member(doc, "rear_axle_given_front_axle", "doc"),
                geo::compute_rear_axle(specs, fa),
                "rear_axle_given_front_axle");

    // trail: dict order drives the comparison vector.
    const Value trail_doc = member(doc, "trail", "doc");
    const Value::Array &torder =
            array_of(member(trail_doc, "order", "trail"), "trail");
    bt::require(torder.size() == 5, "trail order size");
    const geo::TrailResult tr = geo::compute_trail(specs);
    std::vector<double> tvals;
    for (const Value &k: torder) {
        const std::string &key = string_of(k, "trail");
        if (key == "ground_trail") tvals.push_back(tr.ground_trail);
        else if (key == "mechanical_trail")
            tvals.push_back(tr.mechanical_trail);
        else if (key == "axle_to_crown")
            tvals.push_back(tr.axle_to_crown);
        else if (key == "steer_ground_intersection_x")
            tvals.push_back(tr.steer_ground_intersection_x);
        else if (key == "ground_z") tvals.push_back(tr.ground_z);
        else bt::require(false, "trail order key: " + key);
    }
    expect_bits(member(trail_doc, "values", "trail"), tvals, "trail");

    // fixed frame points: the dict insertion order flattened 9x3.
    const Value ffp_doc = member(doc, "fixed_frame_points", "doc");
    const Value::Array &forder =
            array_of(member(ffp_doc, "order", "ffp"), "ffp");
    bt::require(forder.size() == geo::FixedFramePoints::kCount,
                "fixed point order size");
    const geo::FixedFramePoints pts =
            geo::get_fixed_frame_points(specs);
    const auto pnames = pts.names();
    std::vector<double> pvals;
    for (std::size_t i = 0; i < forder.size(); ++i) {
        bt::require(string_of(forder.at(i), "ffp") == pnames.at(i),
                    "fixed point order name");
        for (const double c: pts.at(i)) pvals.push_back(c);
    }
    expect_bits(member(ffp_doc, "values", "ffp"), pvals,
                "fixed_frame_points");

    // frame angles: dict order drives the comparison vector.
    const Value fa_doc = member(doc, "frame_angles", "doc");
    const Value::Array &aorder =
            array_of(member(fa_doc, "order", "angles"), "angles");
    const geo::FrameAngles ang = geo::compute_frame_angles(specs);
    std::vector<double> avals;
    for (const Value &k: aorder) {
        const std::string &key = string_of(k, "angles");
        if (key == "angle_x_tube1_deg")
            avals.push_back(ang.angle_x_tube1_deg);
        else if (key == "angle_x_tube2_deg")
            avals.push_back(ang.angle_x_tube2_deg);
        else if (key == "angle_seat_tube_x_deg")
            avals.push_back(ang.angle_seat_tube_x_deg);
        else if (key == "effective_seat_tube_angle_deg")
            avals.push_back(ang.effective_seat_tube_angle_deg);
        else if (key == "angle_1_deg")
            avals.push_back(ang.angle_1_deg);
        else if (key == "head_angle_deg")
            avals.push_back(ang.head_angle_deg);
        else bt::require(false, "frame angle order key: " + key);
    }
    expect_bits(member(fa_doc, "values", "angles"), avals,
                "frame_angles");
}

// ---- geometry/cockpit ---------------------------------------------------------

void test_cockpit() {
    const Value doc = doc_of("geometry/cockpit");
    const geo::BikeSpecs specs;
    const geo::FixedFramePoints pts =
            geo::get_fixed_frame_points(specs);
    const geo::Vec3 p9_m{pts.P9[0] / 1000.0, pts.P9[1] / 1000.0,
                         pts.P9[2] / 1000.0};
    const geo::Vec3 p10_m{pts.P10[0] / 1000.0, pts.P10[1] / 1000.0,
                          pts.P10[2] / 1000.0};
    expect_bits(member(doc, "seatpost_axis", "doc"), geo::SEATPOST_AXIS,
                "seatpost_axis");
    expect_bits(member(doc, "p9_m", "doc"), p9_m, "p9_m");
    expect_bits(member(doc, "p10_m", "doc"), p10_m, "p10_m");

    const auto check_geometry = [&](const Value &sect,
                                    const geo::SaddleGeometry &g,
                                    std::string_view label) {
        expect_bits(member(sect, "seatpost_top", label),
                    g.seatpost_top, "seatpost_top");
        expect_bits(member(sect, "saddle_pos", label), g.saddle_pos,
                    "saddle_pos");
        expect_bits(member(sect, "top_center", label), g.top_center,
                    "top_center");
        const std::vector<double> scal{g.top_z_m, g.height_m,
                                       g.exposed_post_m};
        expect_bits(member(sect, "scalars", label), scal,
                    "saddle scalars");
    };
    check_geometry(member(doc, "default", "doc"),
                   geo::saddle_geometry(p9_m), "default");
    const Value explicit_sect = member(doc, "explicit_top_z", "doc");
    check_geometry(
            explicit_sect,
            geo::saddle_geometry(p9_m,
                                 number_of(member(explicit_sect,
                                                  "input_top_z_m",
                                                  "explicit"),
                                           "explicit")),
            "explicit_top_z");

    // Inverse: heights -> saddle_top_z_for_height.
    const Value inv = member(doc, "inverse", "doc");
    const std::vector<double> heights = doubles_of(
            member(inv, "input_heights_m", "inverse"), "inverse");
    std::vector<double> tops;
    tops.reserve(heights.size());
    for (const double h: heights)
        tops.push_back(geo::saddle_top_z_for_height(p9_m, h));
    expect_bits(member(inv, "outputs", "inverse"), tops,
                "inverse tops");

    // Mass split + centre of mass for the three inputs.
    const Value masses = member(doc, "masses", "doc");
    const std::vector<double> m_in = doubles_of(
            member(masses, "inputs_kg", "masses"), "masses");
    const geo::SaddleGeometry g0 = geo::saddle_geometry(p9_m);
    std::vector<double> m_out, coms;
    for (const double m: m_in) {
        const geo::SaddlePostMasses mm =
                geo::saddle_post_masses(m, p10_m, p9_m, g0);
        m_out.insert(m_out.end(),
                     {mm.lower_post, mm.upper_post, mm.saddle});
        const geo::Vec3 c = geo::saddle_post_center_of_mass(m, p10_m,
                                                          p9_m, g0);
        coms.insert(coms.end(), {c[0], c[1], c[2]});
    }
    expect_bits(member(masses, "outputs", "masses"), m_out,
                "post masses");
    expect_bits(member(masses, "centres", "masses"), coms,
                "post centres");

    expect_bits(member(doc, "handlebar_grip_point", "doc"),
                geo::handlebar_grip_point(specs), "grip");

    const Value pedals = member(doc, "pedal_points", "doc");
    const auto ped = geo::pedal_points(
            number_of(member(pedals, "crank_length_m", "pedals"),
                      "pedals"));
    expect_bits(member(pedals, "values", "pedals"),
                std::vector<double>{ped.first[0], ped.first[1],
                                    ped.first[2], ped.second[0],
                                    ped.second[1], ped.second[2]},
                "pedals");

    const Value zcases = member(doc, "seatpost_top_cases", "doc");
    std::vector<double> tops2;
    for (const double z: doubles_of(
                 member(zcases, "input_top_z_m", "zcases"), "zcases")) {
        const geo::Vec3 t = geo::seatpost_top_for_saddle_z(p9_m, z);
        tops2.insert(tops2.end(), {t[0], t[1], t[2]});
    }
    expect_bits(member(zcases, "outputs", "zcases"), tops2,
                "seatpost tops");

    // Unreachable-height ValueError + message.
    const Value herr = member(doc, "height_error", "doc");
    const double bad_h = number_of(member(herr, "input_height_m", "h"),
                                   "h");
    expect_invalid_msg(
            string_of(member(herr, "message", "h"), "h"),
            [&] { (void)geo::saddle_top_z_for_height(p9_m, bad_h); },
            "saddle height error");
}

// ---- geometry/validation ------------------------------------------------------

void apply_field(geo::BikeSpecs &s, const std::string &field,
                 double value) {
    if (field == "reach") s.reach = value;
    else if (field == "stack") s.stack = value;
    else if (field == "head_angle_deg") s.head_angle_deg = value;
    else if (field == "wheelbase") s.wheelbase = value;
    else if (field == "front_wheel_radius")
        s.front_wheel_radius = value;
    else if (field == "rear_wheel_radius")
        s.rear_wheel_radius = value;
    else
        bt::require(false, "unhandled validation field: " + field);
}

void test_validation() {
    const Value doc = doc_of("geometry/validation_cases");
    for (const Value &c: array_of(member(doc, "cases", "doc"), "doc")) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const Value *field_v = member_opt(c, "field", "c");
        geo::BikeSpecs specs;
        if (field_v != nullptr && !io::is_null(*field_v)) {
            apply_field(specs,
                        string_of(*field_v, "c.field"),
                        number_of(member(c, "value", "c"), "c.value"));
        }
        const bool accepted = bool_of(member(c, "accepted", "c"), "c");
        if (accepted) {
            bt::require(geo::validate_geometry(specs),
                        "validation case must accept: " + tag);
        } else {
            expect_invalid_msg(
                    string_of(member(c, "message", "c"), "c"),
                    [&] { (void)geo::validate_geometry(specs); },
                    "validation case " + tag);
        }
    }
}

// ---- kinematics/linkage_math ---------------------------------------------------

const std::function<double(double)> &
brentq_objective(std::string_view tag) {
    static const std::function<double(double)> quad =
            [](double x) { return x * x - 2.0; };
    static const std::function<double(double)> cubic =
            [](double x) { return std::pow(x, 3.0) - x - 2.0; };
    static const std::function<double(double)> ident =
            [](double x) { return x; };
    static const std::function<double(double)> shift =
            [](double x) { return x - 1.0; };
    static const std::function<double(double)> qpos =
            [](double x) { return x * x + 1.0; };
    static const std::function<double(double)> qneg =
            [](double x) { return -(x * x) - 4.0; };
    static const std::function<double(double)> tans =
            [](double x) { return std::tan(x) - 1.0; };
    if (tag == "sqrt2" || tag == "sqrt2_xtol" ||
        tag == "sqrt2_rtol_ignored" || tag == "reversed" ||
        tag == "maxiter_3" || tag == "nan_endpoint")
        return quad;
    if (tag == "cubic") return cubic;
    if (tag == "root_at_a") return ident;
    if (tag == "root_at_b") return shift;
    if (tag == "unbracketed") return qpos;
    if (tag == "unbracketed_neg") return qneg;
    if (tag == "tan_solve") return tans;
    bt::require(false,
                "unhandled brentq objective tag: " + std::string(tag));
    std::abort();
}

void test_linkage() {
    const Value doc = doc_of("kinematics/linkage_cases");

    // brentq: replay each case through the recorded f64 inputs.
    const Value bq_doc = member(doc, "brentq", "doc");
    const Value::Array &cases =
            array_of(member(bq_doc, "cases", "brentq"), "brentq");
    const std::vector<double> inputs = bt::read_f64(
            bt::fixture(array_path(member(bq_doc, "inputs", "brentq"),
                                   "brentq.inputs")));
    bt::require(inputs.size() == cases.size() * 5,
                "brentq input lanes");
    const std::string results_rel = array_path(
            member(bq_doc, "results", "brentq"), "brentq.results");
    const std::vector<double> results =
            bt::read_f64(bt::fixture(results_rel));
    std::size_t res_i = 0;
    for (std::size_t i = 0; i < cases.size(); ++i) {
        const Value &c = cases.at(i);
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const double a = inputs.at(i * 5 + 0);
        const double b = inputs.at(i * 5 + 1);
        const double xtol = inputs.at(i * 5 + 2);
        const double rtol = inputs.at(i * 5 + 3);
        const int maxiter = static_cast<int>(inputs.at(i * 5 + 4));
        const auto &f = brentq_objective(tag);
        if (member_opt(c, "result", "c") != nullptr) {
            const double got = kin::brentq(f, a, b, xtol, rtol, maxiter);
            bt::require(res_i < results.size(), "results slot");
            bt::require(std::bit_cast<std::uint64_t>(got) ==
                            std::bit_cast<std::uint64_t>(
                                    results.at(res_i)),
                        "brentq result bits: " + tag);
            ++res_i;
        } else {
            expect_invalid_msg(
                    string_of(member(c, "message", "c"), "c"),
                    [&] {
                        (void)kin::brentq(f, a, b, xtol, rtol,
                                          maxiter);
                    },
                    "brentq case " + tag);
        }
    }
    bt::require(res_i == results.size(), "brentq results consumed");

    // circle-circle intersection: replay through the recorded lanes —
    // [len1, c1[3], r1, len2, c2[3], r2].
    const Value cc_doc = member(doc, "circles", "doc");
    const Value::Array &ccases =
            array_of(member(cc_doc, "cases", "circles"), "circles");
    const std::vector<double> cinputs = bt::read_f64(
            bt::fixture(array_path(member(cc_doc, "inputs", "circles"),
                                   "circles.inputs")));
    bt::require(cinputs.size() == ccases.size() * 10,
                "circle input lanes");
    const std::vector<double> solutions = bt::read_f64(bt::fixture(
            array_path(member(cc_doc, "solutions", "circles"),
                       "circles.solutions")));
    std::size_t sol_i = 0;
    for (std::size_t i = 0; i < ccases.size(); ++i) {
        const Value &c = ccases.at(i);
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const std::size_t lane = i * 10;
        const int n1 = static_cast<int>(cinputs.at(lane + 0));
        const double r1 = cinputs.at(lane + 4);
        const int n2 = static_cast<int>(cinputs.at(lane + 5));
        const double r2 = cinputs.at(lane + 9);
        // Replay through the span overload so the recorded center
        // lengths drive the same indexing branch the oracle took.
        const geo::Vec2 p1v{cinputs.at(lane + 1), cinputs.at(lane + 2)};
        const geo::Vec3 p1w{cinputs.at(lane + 1), cinputs.at(lane + 2),
                            cinputs.at(lane + 3)};
        const geo::Vec2 p2v{cinputs.at(lane + 6), cinputs.at(lane + 7)};
        const geo::Vec3 p2w{cinputs.at(lane + 6), cinputs.at(lane + 7),
                            cinputs.at(lane + 8)};
        const auto call = [&]() -> std::pair<geo::Vec2, geo::Vec2> {
            if (n1 == 3)
                return kin::circle_circle_intersection_2d(
                        std::span<const double>(p1w), r1,
                        n2 == 3 ? std::span<const double>(p2w)
                                : std::span<const double>(p2v),
                        r2);
            return kin::circle_circle_intersection_2d(
                    std::span<const double>(p1v), r1,
                    n2 == 3 ? std::span<const double>(p2w)
                            : std::span<const double>(p2v),
                    r2);
        };
        if (member_opt(c, "error", "c") == nullptr) {
            const auto [s1, s2] = call();
            bt::require(sol_i + 4 <= solutions.size(),
                        "solutions slot");
            const std::array<double, 4> got{s1[0], s1[1], s2[0], s2[1]};
            for (std::size_t k = 0; k < 4; ++k)
                bt::require(
                        std::bit_cast<std::uint64_t>(got.at(k)) ==
                                std::bit_cast<std::uint64_t>(
                                        solutions.at(sol_i + k)),
                        "circle solution bits: " + tag);
            sol_i += 4;
        } else {
            expect_invalid_msg(
                    string_of(member(c, "message", "c"), "c"),
                    [&] { (void)call(); }, "circle case " + tag);
        }
    }
    bt::require(sol_i == solutions.size(),
                "circle solutions consumed");
}

// ---- kinematics/solver ------------------------------------------------------------

// The scalar_order members of a state dict, in order.
std::vector<double> state_scalars(const kin::KinematicState &s) {
    return {s.wheel_travel,           s.shock_stroke,
            s.shock_length,           s.theta_cs,
            s.theta_cs_deg,           s.transmission_angle_deg,
            s.leverage_ratio,         s.max_link_error};
}
const geo::Vec3 &state_point(const kin::KinematicState &s,
                             std::string_view name) {
    if (name == "P0") return s.P0;
    if (name == "P1") return s.P1;
    if (name == "P2") return s.P2;
    if (name == "P3") return s.P3;
    if (name == "P4") return s.P4;
    if (name == "P5") return s.P5;
    if (name == "P6") return s.P6;
    if (name == "P7") return s.P7;
    if (name == "P12") return s.P12;
    if (name == "P_FA") return s.P_FA;
    if (name == "P11") return s.P11;
    if (name == "BB") return s.BB;
    bt::require(false,
                "unhandled state point name: " + std::string(name));
    std::abort();
}

// Flat copy of a Vec3 array — the row-major (n, 3) fixture layout.
std::vector<double> flat(const std::vector<geo::Vec3> &v) {
    std::vector<double> out;
    out.reserve(v.size() * 3);
    for (const geo::Vec3 &p: v)
        for (const double c: p) out.push_back(c);
    return out;
}

// Link-length residuals re-derived from the produced pivots and the
// solver's public invariants — bitwise equal to the state's own
// link_errors members and inside the physical bound the linkage holds.
void check_link_residuals(const kin::KinematicState &state,
                          const kin::HorstLinkageSolver &solver,
                          std::string_view label) {
    const auto sub = [](const geo::Vec3 &a, const geo::Vec3 &b) {
        return geo::Vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    };
    const kin::LinkErrors recomputed{
            .chainstay = std::fabs(
                    geo::detail::np_norm3(sub(state.P2, solver.p0())) -
                    solver.l_cs()),
            .seatstay = std::fabs(
                    geo::detail::np_norm3(sub(state.P3, state.P2)) -
                    solver.l_ss()),
            .rocker_p5_p3 = std::fabs(
                    geo::detail::np_norm3(sub(state.P3, solver.p5())) -
                    solver.r_53()),
            .rocker_p5_p4 = std::fabs(
                    geo::detail::np_norm3(sub(state.P4, solver.p5())) -
                    solver.r_54()),
            .axle_offset = std::fabs(
                    geo::detail::np_norm3(sub(state.P1, state.P2)) -
                    solver.r_21()),
            .dropout_upper = std::fabs(
                    geo::detail::np_norm3(sub(state.P12, state.P2)) -
                    solver.r_2_12()),
            .dropout_height = std::fabs(
                    geo::detail::np_norm3(sub(state.P1, state.P12)) -
                    solver.l_dropout_height()),
            .yoke = std::fabs(
                    geo::detail::np_norm3(sub(state.P6, state.P4)) -
                    solver.l_yoke())};
    const std::array<double, 8> re{recomputed.chainstay,
                                   recomputed.seatstay,
                                   recomputed.rocker_p5_p3,
                                   recomputed.rocker_p5_p4,
                                   recomputed.axle_offset,
                                   recomputed.dropout_upper,
                                   recomputed.dropout_height,
                                   recomputed.yoke};
    const std::array<double, 8> st{state.link_errors.chainstay,
                                   state.link_errors.seatstay,
                                   state.link_errors.rocker_p5_p3,
                                   state.link_errors.rocker_p5_p4,
                                   state.link_errors.axle_offset,
                                   state.link_errors.dropout_upper,
                                   state.link_errors.dropout_height,
                                   state.link_errors.yoke};
    for (std::size_t k = 0; k < 8; ++k) {
        bt::require(std::bit_cast<std::uint64_t>(re.at(k)) ==
                        std::bit_cast<std::uint64_t>(st.at(k)),
                    std::string(label) +
                            " link_errors member mismatch");
        bt::require(st.at(k) < 1e-9,
                    std::string(label) + " link residual bound");
    }
    bt::require(std::bit_cast<std::uint64_t>(recomputed.max()) ==
                    std::bit_cast<std::uint64_t>(state.max_link_error),
                std::string(label) + " max_link_error fold");
}

void test_solver_inverse() {
    const Value doc = doc_of("kinematics/stroke_inverse");
    // The declared orders pin the array layout before any compare.
    static const std::array<std::string_view, 8> kScalarOrder{
            "wheel_travel",           "shock_stroke",
            "shock_length",           "theta_cs",
            "theta_cs_deg",           "transmission_angle_deg",
            "leverage_ratio",         "max_link_error"};
    static const std::array<std::string_view, 12> kPointOrder{
            "P0", "P1", "P2", "P3",  "P4",  "P5",
            "P6", "P7", "P12", "P_FA", "P11", "BB"};
    const Value::Array &scalar_order = array_of(
            member(doc, "scalar_order", "doc"), "scalar_order");
    const Value::Array &point_order = array_of(
            member(doc, "point_order", "doc"), "point_order");
    bt::require(scalar_order.size() == 8 && point_order.size() == 12,
                "solver orders");
    for (std::size_t k = 0; k < 8; ++k)
        bt::require(string_of(scalar_order.at(k), "scalar_order") ==
                        kScalarOrder.at(k),
                    "scalar_order name");
    for (std::size_t k = 0; k < 12; ++k)
        bt::require(string_of(point_order.at(k), "point_order") ==
                        kPointOrder.at(k),
                    "point_order name");

    const kin::HorstLinkageSolver solver;

    // Row i of a (n,8) scalars ref and each (n,3) point ref is checked
    // bitwise against the produced state.
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (row index, row count) call sites stay ordered by usage
    const auto check_state_rows = [&](const Value &sect, std::size_t i,
                                      std::size_t n,
                                      const kin::KinematicState &st,
                                      std::string_view label) {
        const std::vector<double> scal = state_scalars(st);
        const std::vector<double> all = bt::read_f64(
                bt::fixture(array_path(member(sect, "scalars", label),
                                       label)));
        bt::require(all.size() == n * 8,
                    std::string(label) + " scalars shape");
        for (std::size_t k = 0; k < 8; ++k)
            bt::require(
                    std::bit_cast<std::uint64_t>(scal.at(k)) ==
                            std::bit_cast<std::uint64_t>(
                                    all.at(i * 8 + k)),
                    std::string(label) + " scalar bits");
        for (const std::string_view name: kPointOrder) {
            const std::vector<double> pts = bt::read_f64(
                    bt::fixture(array_path(member(sect, name, label),
                                           label)));
            bt::require(pts.size() == n * 3,
                        std::string(label) + " point shape");
            const geo::Vec3 &got = state_point(st, name);
            for (std::size_t k = 0; k < 3; ++k)
                bt::require(
                        std::bit_cast<std::uint64_t>(got.at(k)) ==
                                std::bit_cast<std::uint64_t>(
                                        pts.at(i * 3 + k)),
                        std::string(label) + " point bits " +
                                std::string(name));
        }
    };

    // solve_state_from_shock_stroke — the clip + np.interp inverse.
    const std::vector<double> strokes = doubles_of(
            member(doc, "stroke_inputs", "doc"), "strokes");
    const Value st_sect = member(doc, "stroke_states", "doc");
    for (std::size_t i = 0; i < strokes.size(); ++i) {
        const kin::KinematicState st =
                solver.solve_state_from_shock_stroke(strokes.at(i));
        check_state_rows(st_sect, i, strokes.size(), st,
                         "stroke state");
        check_link_residuals(st, solver, "stroke state");
    }

    // solve_state_from_wheel_travel — the brentq inverse.
    const std::vector<double> travels = doubles_of(
            member(doc, "travel_inputs", "doc"), "travels");
    const Value ts_sect = member(doc, "travel_states", "doc");
    for (std::size_t i = 0; i < travels.size(); ++i) {
        const kin::KinematicState st =
                solver.solve_state_from_wheel_travel(travels.at(i));
        check_state_rows(ts_sect, i, travels.size(), st,
                         "travel state");
    }

    // solve_state_from_chainstay_angle at the construction pose.
    const kin::KinematicState s0 =
            solver.solve_state_from_chainstay_angle(
                    solver.theta_cs_0());
    check_state_rows(member(doc, "theta_cs_0_state", "doc"), 0, 1, s0,
                     "theta0 state");
    check_link_residuals(s0, solver, "theta0 state");

    // _stroke_table contents: 101 wheel-travel solves reproduce the
    // interp table the stroke solve consumes (endpoint-clipped
    // queries travel the endpoint branch — the table itself is what
    // the oracle's np.interp sees).
    const Value table = member(doc, "stroke_table", "doc");
    const std::vector<double> tv_expected = bt::read_f64(bt::fixture(
            array_path(member(table, "travel", "tbl"), "tbl.t")));
    const std::vector<double> ts_expected = bt::read_f64(bt::fixture(
            array_path(member(table, "stroke", "tbl"), "tbl.s")));
    const std::vector<double> tv = geo::detail::np_linspace(
            0.0, solver.specs().rear_wheel_travel, tv_expected.size());
    bt::require(tv.size() == tv_expected.size(), "table travel size");
    for (std::size_t i = 0; i < tv.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(tv.at(i)) ==
                        std::bit_cast<std::uint64_t>(tv_expected.at(i)),
                    "stroke table travel bits");
    std::vector<double> ts;
    ts.reserve(tv.size());
    for (const double t: tv)
        ts.push_back(
                solver.solve_state_from_wheel_travel(t).shock_stroke);
    bt::require(ts.size() == ts_expected.size(), "table stroke size");
    for (std::size_t i = 0; i < ts.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(ts.at(i)) ==
                        std::bit_cast<std::uint64_t>(ts_expected.at(i)),
                    "stroke table stroke bits");
}

// ---- kinematics/curves -----------------------------------------------------------

void test_instant_centers() {
    const Value doc = doc_of("kinematics/instant_centers");
    const int n = static_cast<int>(
            number_of(member(doc, "n_points", "doc"), "doc"));
    const kin::HorstLinkageSolver solver;
    const kin::InstantCenters ic =
            kin::compute_instant_centers(solver, n);
    expect_bits(member(doc, "travel", "doc"), ic.travel, "travel");
    expect_bits(member(doc, "ic_x", "doc"), ic.ic_x, "ic_x");
    expect_bits(member(doc, "ic_z", "doc"), ic.ic_z, "ic_z");
}

// ---- kinematics/trajectory (dense fixture) ----------------------------------------

void test_trajectory() {
    const Value doc = doc_of("kinematics/trajectory");
    const kin::HorstLinkageSolver solver;
    const kin::KinematicTrajectory t = solver.solve_trajectory(101,
                                                             180.0);

    expect_bits(member(doc, "wheel_travel", "traj"), t.wheel_travel,
                "wheel_travel");
    expect_bits(member(doc, "shock_stroke", "traj"),
                t.shock_stroke, "shock_stroke");
    expect_bits(member(doc, "shock_length", "traj"),
                t.shock_length, "shock_length");
    expect_bits(member(doc, "leverage_ratio", "traj"),
                t.leverage_ratio, "leverage_ratio");
    expect_bits(member(doc, "transmission_angle_deg", "traj"),
                t.transmission_angle_deg,
                "transmission_angle_deg");
    expect_bits(member(doc, "axle_path_x", "traj"), t.axle_path_x,
                "axle_path_x");
    expect_bits(member(doc, "axle_path_z", "traj"), t.axle_path_z,
                "axle_path_z");
    expect_bits(member(doc, "chainstay_angle_deg", "traj"),
                t.chainstay_angle_deg, "chainstay_angle_deg");
    expect_bits(member(doc, "link_length_errors", "traj"),
                t.link_length_errors, "link_length_errors");

    // Scalar block (JSON numbers — exact repr decode).
    const auto scalar_eq = [&](const char *key, double got) {
        bt::require(
                std::bit_cast<std::uint64_t>(got) ==
                        std::bit_cast<std::uint64_t>(
                                scalar_of(member(doc, key, "traj"),
                                          key)),
                std::string("trajectory scalar: ") + key);
    };
    scalar_eq("max_link_error", t.max_link_error);
    scalar_eq("initial_leverage_ratio", t.initial_leverage_ratio);
    scalar_eq("final_leverage_ratio", t.final_leverage_ratio);
    scalar_eq("progressivity_pct", t.progressivity_pct);
    scalar_eq("max_stroke", t.max_stroke);

    // Point trajectories in the declared point order.
    for (const std::string_view name:
             kin::KinematicTrajectory::kPointNames)
        expect_bits(member(doc, name, "traj"),
                    flat(t.point_array(name)),
                    std::string("traj ") + std::string(name));

    // Every embedded state dict: scalars + link_errors as exact JSON
    // repr numbers, the twelve points as .f64 references.
    const Value::Array &states =
            array_of(member(doc, "states", "traj"), "states");
    bt::require(states.size() == t.states.size(), "states count");
    static const std::array<std::string_view, 8> kScalarKeys{
            "wheel_travel",       "shock_stroke",
            "shock_length",       "theta_cs",
            "theta_cs_deg",       "transmission_angle_deg",
            "leverage_ratio",     "max_link_error"};
    static const std::array<std::string_view, 8> kErrKeys{
            "chainstay",    "seatstay",      "rocker_p5_p3",
            "rocker_p5_p4", "axle_offset",   "dropout_upper",
            "dropout_height", "yoke"};
    for (std::size_t i = 0; i < states.size(); ++i) {
        const Value &sd = states.at(i);
        const kin::KinematicState &st = t.states.at(i);
        const std::array<double, 8> got_scal{
                st.wheel_travel,  st.shock_stroke,
                st.shock_length,  st.theta_cs,
                st.theta_cs_deg,
                st.transmission_angle_deg,
                st.leverage_ratio,
                st.max_link_error};
        for (std::size_t k = 0; k < 8; ++k)
            bt::require(
                    std::bit_cast<std::uint64_t>(got_scal.at(k)) ==
                            std::bit_cast<std::uint64_t>(scalar_of(
                                    member(sd, kScalarKeys.at(k),
                                           "state"),
                                    "state")),
                    "state scalar " +
                            std::string(kScalarKeys.at(k)));
        const Value le = member(sd, "link_errors", "state");
        const std::array<double, 8> got_err{
                st.link_errors.chainstay,
                st.link_errors.seatstay,
                st.link_errors.rocker_p5_p3,
                st.link_errors.rocker_p5_p4,
                st.link_errors.axle_offset,
                st.link_errors.dropout_upper,
                st.link_errors.dropout_height,
                st.link_errors.yoke};
        for (std::size_t k = 0; k < 8; ++k)
            bt::require(
                    std::bit_cast<std::uint64_t>(got_err.at(k)) ==
                            std::bit_cast<std::uint64_t>(scalar_of(
                                    member(le, kErrKeys.at(k),
                                           "link_errors"),
                                    "link_errors")),
                    "link error " + std::string(kErrKeys.at(k)));
        const std::array<geo::Vec3, 12> pts{
                st.P0, st.P1, st.P2, st.P3,
                st.P4, st.P5, st.P6, st.P7,
                st.P12, st.P_FA, st.P11, st.BB};
        static const std::array<std::string_view, 12> kPtKeys{
                "P0", "P1", "P2", "P3", "P4", "P5",
                "P6", "P7", "P12", "P_FA", "P11", "BB"};
        for (std::size_t k = 0; k < 12; ++k)
            expect_bits(member(sd, kPtKeys.at(k), "state"), pts.at(k),
                        "state point");
    }
}

} // namespace

int main() {
    return bt::run([] {
        test_specs_defaults();
        test_hardpoints();
        test_cockpit();
        test_validation();
        test_linkage();
        test_solver_inverse();
        test_instant_centers();
        test_trajectory();
    });
}
