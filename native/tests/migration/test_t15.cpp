// test_t15.cpp — T15 model-boundary migration regressions. Covers the
// owned declarations behind the T15 boundary: inertia.py's ring/
// parallel-axis/wheel tensors and MJCF inertial insertion, component_
// masses.py's geom weight distribution + registry, mass.py's
// BikeMassSpecs budgets / component centres / static system CG, rider.py's
// resolution layer / seated pose solver / leg chains / apparent mass /
// path masses, rider_segments.py's De Leva tables / arm split / segment
// inertia / articulated pose, air_spring.py's ForkAirSpring, coil_
// shock.py's CoilShock, damper.py's presets / Charger3 / SuperDeluxe /
// suspension system, tuning.py's reflected mass + sag rollup, and
// sim/ride/mass_properties.py's compiled-model reads — every comparison
// bitwise against the oracle fixtures declared in the manifest
// (comparison B), with the fixture docs driving the replay inputs.
#include "support/migration_test.hpp"

#include "configuration/rider_spec.hpp"
#include "engine_call.hpp"
#include "geometry/specs.hpp"
#include "io/json.hpp"
#include "model/mass.hpp"
#include "model/rider_geometry.hpp"
#include "model/suspension_setup.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

namespace cfg = bike::configuration;
namespace geo = bike::geometry;
namespace io = bike::io;
namespace mdl = bike::model;
using io::Value;

// ---- JSON helpers (same idioms as test_t14) ---------------------------------

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
double number_of(const Value &value) {
    return number_of(value, "number");
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

// A manifest-validated JSON doc: the entry's declared files verify
// (bytes + SHA-256) before the doc text is accepted.
Value doc_of(std::string_view name) {
    bt::manifest_verify_files(bt::manifest_entry(name));
    return io::parse_json(
            bt::text(bt::fixture(std::string(name) + ".json")));
}

// ---- canonical-tag decoders --------------------------------------------------

// $array reference: validate dtype/order, return the fixture path.
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
// Bitwise binary64 comparison against the referenced golden .f64.
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
void expect_bits(const Value &ref, const mdl::Mat3 &actual,
                 std::string_view label) {
    expect_bits(ref, std::span<const double>(actual), label);
}

// A JSON scalar that may carry the $nonfinite tag.
double scalar_field(const Value &v, std::string_view label) {
    if (std::get_if<Value::Object>(&v.data) == nullptr)
        return number_of(v, label);
    if (const Value *nf = member_opt(v, "$nonfinite", label)) {
        const std::string &kind = string_of(*nf, label);
        if (kind == "nan")
            return std::numeric_limits<double>::quiet_NaN();
        if (kind == "inf")
            return std::numeric_limits<double>::infinity();
        bt::require(kind == "-inf",
                    std::string(label) + ": bad $nonfinite kind");
        return -std::numeric_limits<double>::infinity();
    }
    return number_of(v, label);
}
// $tuple / plain list of scalars.
std::vector<double> scalar_list(const Value &v, std::string_view label) {
    std::vector<double> out;
    const Value *items = &v;
    if (std::get_if<Value::Object>(&v.data) != nullptr)
        if (const Value *t = member_opt(v, "$tuple", label)) items = t;
    for (const Value &item: array_of(*items, label))
        out.push_back(scalar_field(item, label));
    return out;
}
bool same_bits(double actual, double expected) {
    return std::bit_cast<std::uint64_t>(actual) ==
           std::bit_cast<std::uint64_t>(expected);
}
void expect_scalar_bits(const Value &expected, double actual,
                        std::string_view label) {
    bt::require(same_bits(actual, scalar_field(expected, label)),
                std::string(label) + ": binary64 mismatch");
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
void expect_key_error(std::string_view expected,
                      const std::function<void()> &action,
                      std::string_view label) {
    try {
        action();
    } catch (const std::out_of_range &error) {
        bt::require(expected == error.what(),
                    std::string(label) + ": KeyError payload mismatch: '" +
                            std::string(error.what()) + "'");
        return;
    }
    bt::require(false,
                std::string(label) + ": expected std::out_of_range");
}

// {accepted, result|error} rows — the outcome() envelope the capture
// group wrote for every callable. The ValueError rows replay through
// std::invalid_argument; the TypeError/typed-domain rows are skipped by
// the caller before reaching this helper (the C++ boundary is typed, so
// those inputs are unrepresentable by construction).
bool outcome_accepted(const Value &row, std::string_view label) {
    return bool_of(member(row, "accepted", label), label);
}
const Value *outcome_result(const Value &row, std::string_view label) {
    return member_opt(row, "result", label);
}
std::string outcome_message(const Value &row, std::string_view label) {
    return string_of(member(row, "message", label), label);
}

// Ordered [[key, value], ...] pair lists (insertion-ordered mappings).
struct PairExpectation {
    std::vector<std::string> names;
    std::vector<double> values;
};
PairExpectation pairs_of(const Value &pairs, std::string_view label) {
    PairExpectation out;
    for (const Value &pair: array_of(pairs, label)) {
        const Value::Array &kv = array_of(pair, label);
        bt::require(kv.size() == 2, std::string(label) + ": pair arity");
        out.names.push_back(string_of(kv.at(0), label));
        out.values.push_back(scalar_field(kv.at(1), label));
    }
    return out;
}
void expect_pairs(const Value &pairs,
                  const std::vector<std::pair<std::string, double>> &actual,
                  std::string_view label) {
    const PairExpectation want = pairs_of(pairs, label);
    bt::require(actual.size() == want.names.size(),
                std::string(label) + ": pair count");
    for (std::size_t i = 0; i < want.names.size(); ++i) {
        bt::require(actual.at(i).first == want.names.at(i),
                    std::string(label) + ": key order mismatch at " +
                            std::to_string(i));
        bt::require(same_bits(actual.at(i).second, want.values.at(i)),
                    std::string(label) + ": bits mismatch at key " +
                            want.names.at(i));
    }
}
// Replay an accepted rows' {names, positions$array, masses$array}
// centres doc against an ordered centre vector.
void expect_centers(const Value &result,
                    const std::vector<mdl::ComponentCenter> &actual,
                    std::string_view label) {
    const Value::Array &names =
            array_of(member(result, "names", label), label);
    bt::require(actual.size() == names.size(),
                std::string(label) + ": centre count");
    std::vector<double> positions, masses;
    positions.reserve(actual.size() * 3);
    masses.reserve(actual.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {
        bt::require(actual.at(i).name ==
                            string_of(names.at(i), label),
                    std::string(label) + ": centre name order at " +
                            std::to_string(i));
        positions.insert(positions.end(), actual.at(i).pos_m.begin(),
                         actual.at(i).pos_m.end());
        masses.push_back(actual.at(i).mass_kg);
    }
    expect_bits(member(result, "positions", label), positions,
                std::string(label) + ".positions");
    expect_bits(member(result, "masses", label), masses,
                std::string(label) + ".masses");
}
void expect_centers(const Value &result,
                    const std::vector<mdl::RiderCenter> &actual,
                    std::string_view label) {
    const Value::Array &names =
            array_of(member(result, "names", label), label);
    bt::require(actual.size() == names.size(),
                std::string(label) + ": centre count");
    std::vector<double> positions, masses;
    positions.reserve(actual.size() * 3);
    masses.reserve(actual.size());
    for (std::size_t i = 0; i < actual.size(); ++i) {
        bt::require(actual.at(i).name ==
                            string_of(names.at(i), label),
                    std::string(label) + ": centre name order at " +
                            std::to_string(i));
        positions.insert(positions.end(), actual.at(i).pos_m.begin(),
                         actual.at(i).pos_m.end());
        masses.push_back(actual.at(i).mass_kg);
    }
    expect_bits(member(result, "positions", label), positions,
                std::string(label) + ".positions");
    expect_bits(member(result, "masses", label), masses,
                std::string(label) + ".masses");
}

geo::Vec3 vec_of(const Value &ref, std::string_view label) {
    const std::vector<double> lane = bt::read_f64(
            bt::fixture(array_path(ref, label)));
    bt::require(lane.size() == 3, std::string(label) + ": vec3 lane");
    return {lane.at(0), lane.at(1), lane.at(2)};
}

// ---- model/inertia ------------------------------------------------------------

void test_inertia() {
    const Value doc = doc_of("model/inertia");

    const Value::Array &rings =
            array_of(member(doc, "ring_cases", "doc"), "doc");
    for (const Value &c: rings) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const Value::Array &args = array_of(member(c, "args", "c"), "c");
        const double m = scalar_field(args.at(0), "args");
        const double ri = scalar_field(args.at(1), "args");
        const double ro = scalar_field(args.at(2), "args");
        const double w = scalar_field(args.at(3), "args");
        if (outcome_accepted(c, "ring " + tag)) {
            expect_bits(*outcome_result(c, "ring " + tag),
                        mdl::ring_inertia(m, ri, ro, w), "ring " + tag);
        } else {
            expect_invalid_msg(
                    outcome_message(c, "ring " + tag),
                    [&] { (void)mdl::ring_inertia(m, ri, ro, w); },
                    "ring " + tag);
        }
    }

    const Value::Array &axes =
            array_of(member(doc, "axis_cases", "doc"), "doc");
    for (const Value &c: axes) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const double m =
                scalar_field(member(c, "mass_kg", "c"), "axis " + tag);
        const geo::Vec3 off =
                vec_of(member(c, "offset_m", "c"), "axis " + tag);
        if (outcome_accepted(c, "axis " + tag)) {
            expect_bits(*outcome_result(c, "axis " + tag),
                        mdl::parallel_axis(m, off), "axis " + tag);
        } else {
            expect_invalid_msg(
                    outcome_message(c, "axis " + tag),
                    [&] { (void)mdl::parallel_axis(m, off); },
                    "axis " + tag);
        }
    }

    // wheel_body_inertia: the authored component tuples by tag.
    const std::vector<mdl::WheelComponent> bad_fraction{
            mdl::WheelComponent{.mass_fraction = 0.5, .inner_radius_m = 0.0,
                                .outer_radius_m = 0.1, .width_m = 0.05,
                                .offset_m = {0.0, 0.0, 0.0}},
            mdl::WheelComponent{.mass_fraction = 0.4, .inner_radius_m = 0.0,
                                .outer_radius_m = 0.05, .width_m = 0.05,
                                .offset_m = {0.0, 0.0, 0.0}}};
    const std::vector<mdl::WheelComponent> neg_fraction{
            mdl::WheelComponent{.mass_fraction = -0.5, .inner_radius_m = 0.0,
                                .outer_radius_m = 0.1, .width_m = 0.05,
                                .offset_m = {0.0, 0.0, 0.0}}};
    const std::vector<mdl::WheelComponent> nan_offset{
            mdl::WheelComponent{.mass_fraction = 1.0, .inner_radius_m = 0.0,
                                .outer_radius_m = 0.1, .width_m = 0.05,
                                .offset_m = {std::numeric_limits<
                                                    double>::quiet_NaN(),
                                            0.0, 0.0}}};
    // wheel_body_inertia: the cases record only the tag, so the authored
    // component tuples/masses replay from the tag dispatch.
    const Value::Array &wheels =
            array_of(member(doc, "wheel_cases", "doc"), "doc");
    for (const Value &c: wheels) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        std::span<const mdl::WheelComponent> comps;
        if (tag == "front" || tag == "zero_mass")
            comps = std::span<const mdl::WheelComponent>(
                    mdl::kFrontWheelProfile);
        else if (tag == "rear")
            comps = std::span<const mdl::WheelComponent>(
                    mdl::kRearWheelProfile);
        else if (tag == "bad_fraction_sum")
            comps = bad_fraction;
        else if (tag == "empty")
            comps = {};
        else if (tag == "neg_fraction")
            comps = neg_fraction;
        else if (tag == "nan_offset")
            comps = nan_offset;
        else
            bt::require(false, "wheel tag: " + tag);
        const double wheel_mass = tag == "front"     ? 2.40
                                  : tag == "rear"    ? 2.80
                                  : tag == "zero_mass" ? 0.0
                                                       : 1.0;
        if (outcome_accepted(c, "wheel " + tag)) {
            const mdl::WheelInertia got =
                    mdl::wheel_body_inertia(wheel_mass, comps);
            const Value &res = *outcome_result(c, "wheel " + tag);
            expect_bits(member(res, "com", "wheel"), got.com,
                        "wheel com");
            expect_bits(member(res, "tensor", "wheel"), got.tensor,
                        "wheel tensor");
        } else {
            expect_invalid_msg(
                    outcome_message(c, "wheel " + tag),
                    [&] { (void)mdl::wheel_body_inertia(wheel_mass, comps); },
                    "wheel " + tag);
        }
    }

    // add_body_inertial: the cases record only the tag — the mass/com/
    // tensor/dup inputs replay from the authored dispatch. Accepted row
    // serialises the .17g attrs; rejected rows carry the validation
    // message.
    const mdl::Mat3 good =
            mdl::ring_inertia(0.5, 0.30, 0.36, 0.05);
    const Value::Array &inert =
            array_of(member(doc, "inertial_cases", "doc"), "doc");
    for (const Value &c: inert) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        double mass = 0.5;
        geo::Vec3 com{0.1, 0.0, 0.2};
        bool dup = false;
        mdl::Mat3 tensor = good;
        if (tag == "duplicate_inertial") {
            com = {0.0, 0.0, 0.0};
            dup = true;
        } else if (tag == "zero_mass") {
            mass = 0.0;
            com = {0.0, 0.0, 0.0};
        } else if (tag == "nonfinite_com") {
            com = {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
        } else if (tag == "asymmetric") {
            com = {0.0, 0.0, 0.0};
            tensor = {1.0, 0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0, 1.0};
        } else if (tag == "indefinite") {
            com = {0.0, 0.0, 0.0};
            tensor = {-1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0};
        } else if (tag == "triangle_violation") {
            com = {0.0, 0.0, 0.0};
            tensor = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 3.0};
        } else {
            bt::require(tag == "accepted",
                        "inertial tag: " + tag);
        }
        const auto run = [&]() -> mdl::XmlBodyElement {
            mdl::XmlBodyElement body;
            if (dup)
                body.inertial = mdl::XmlInertialAttributes{.mass = "1.0",
                                                         .pos = "",
                                                         .fullinertia = ""};
            mdl::add_body_inertial(body, mass, com, tensor);
            return body;
        };
        if (outcome_accepted(c, "inertial " + tag)) {
            const mdl::XmlBodyElement body = run();
            const Value &res = *outcome_result(c, "inertial " + tag);
            if (body.inertial.has_value()) {
                const mdl::XmlInertialAttributes &inertial =
                        *body.inertial;
                bt::require(inertial.mass ==
                                string_of(member(res, "mass", "res"),
                                          "res"),
                            "inertial mass attr " + tag);
                bt::require(inertial.pos ==
                                string_of(member(res, "pos", "res"),
                                          "res"),
                            "inertial pos attr " + tag);
                bt::require(inertial.fullinertia ==
                                string_of(member(res, "fullinertia",
                                                 "res"),
                                          "res"),
                            "inertial fullinertia attr " + tag);
            } else {
                bt::require(false, "inertial written");
            }
        } else {
            expect_invalid_msg(outcome_message(c, "inertial " + tag),
                               [&] { (void)run(); },
                               "inertial " + tag);
        }
    }
}

// ---- model/component_masses ---------------------------------------------------

void test_component_masses() {
    const Value doc = doc_of("model/component_masses");

    const Value::Array &assigns =
            array_of(member(doc, "assign_cases", "doc"), "doc");
    for (const Value &c: assigns) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const double total =
                scalar_field(member(c, "total_kg", "c"), "assign " + tag);
        std::vector<mdl::XmlGeomElement> geoms;
        for (const Value &w: array_of(member(c, "weights", "c"), "c")) {
            mdl::XmlGeomElement g;
            if (!io::is_null(w)) g.mass = string_of(w, "weight");
            geoms.push_back(std::move(g));
        }
        if (outcome_accepted(c, "assign " + tag)) {
            mdl::assign_component_mass(geoms, total);
            const Value::Array &res = array_of(
                    *outcome_result(c, "assign " + tag), "assign " + tag);
            bt::require(res.size() == geoms.size(), "assign arity");
            for (std::size_t i = 0; i < res.size(); ++i) {
                const std::optional<std::string> &mass = geoms.at(i).mass;
                if (mass.has_value())
                    bt::require(*mass == string_of(res.at(i), "mass"),
                                "assign .17g attr " + tag + " #" +
                                        std::to_string(i));
                else
                    bt::require(false,
                                "assign .17g attr " + tag + " #" +
                                        std::to_string(i));
            }
        } else {
            expect_invalid_msg(
                    outcome_message(c, "assign " + tag),
                    [&] { mdl::assign_component_mass(geoms, total); },
                    "assign " + tag);
        }
    }

    // register_component: the registry is shared across rows exactly
    // like the oracle's dict. The registry stores element pointers, so
    // the per-case vectors live in a deque for the whole loop.
    mdl::ComponentRegistry registry;
    std::deque<std::vector<mdl::XmlGeomElement>> storage;
    const Value::Array &regs =
            array_of(member(doc, "register_cases", "doc"), "doc");
    for (const Value &c: regs) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const std::string &cid =
                string_of(member(c, "component_id", "c"), "c");
        std::vector<mdl::XmlGeomElement> &parts = storage.emplace_back();
        for (const Value &w: array_of(member(c, "masses", "c"), "c")) {
            mdl::XmlGeomElement g;
            if (!io::is_null(w)) g.mass = string_of(w, "mass");
            parts.push_back(std::move(g));
        }
        if (outcome_accepted(c, "register " + tag)) {
            mdl::register_component(registry, cid, parts);
            const Value &res = *outcome_result(c, "register " + tag);
            const Value::Array &ids =
                    array_of(member(res, "ids", "res"), "res");
            const Value::Array &sizes =
                    array_of(member(res, "sizes", "res"), "res");
            bt::require(registry.size() == ids.size(),
                        "registry size " + tag);
            for (std::size_t i = 0; i < ids.size(); ++i) {
                bt::require(registry.at(i).first ==
                                string_of(ids.at(i), "id"),
                            "registry id order " + tag);
                bt::require(registry.at(i).second.size() ==
                                static_cast<std::size_t>(number_of(
                                        sizes.at(i), "size")),
                            "registry lane size " + tag);
            }
        } else {
            expect_invalid_msg(
                    outcome_message(c, "register " + tag),
                    [&] { mdl::register_component(registry, cid, parts); },
                    "register " + tag);
        }
    }
}

// ---- model/mass_specs -----------------------------------------------------------

void test_mass_specs() {
    const Value doc = doc_of("model/mass_specs");
    const mdl::BikeMassSpecs specs;
    const std::vector<std::pair<std::string, double>> comp =
            specs.component_masses();
    const Value::Array &names =
            array_of(member(doc, "names", "doc"), "doc");
    bt::require(comp.size() == names.size(), "component count");
    std::vector<double> masses;
    for (std::size_t i = 0; i < comp.size(); ++i) {
        bt::require(comp.at(i).first == string_of(names.at(i), "name"),
                    "component_masses key order at " + std::to_string(i));
        masses.push_back(comp.at(i).second);
    }
    expect_bits(member(doc, "masses", "doc"), masses, "component masses");

    const Value::Array &prov =
            array_of(member(doc, "provenance", "doc"), "doc");
    const auto prov_got = specs.component_provenance();
    bt::require(prov.size() == prov_got.size(), "provenance count");
    for (std::size_t i = 0; i < prov.size(); ++i) {
        const Value::Array &row = array_of(prov.at(i), "prov");
        bt::require(row.size() == 3, "provenance arity");
        bt::require(prov_got.at(i).first == string_of(row.at(0), "p"),
                    "provenance id order");
        bt::require(prov_got.at(i).second.mass ==
                            string_of(row.at(1), "p"),
                    "provenance mass source");
        bt::require(prov_got.at(i).second.distribution ==
                            string_of(row.at(2), "p"),
                    "provenance distribution source");
    }
    expect_scalar_bits(member(doc, "total_bike_mass", "doc"),
                       specs.total_bike_mass(), "total_bike_mass");

    const Value::Array &wheels = array_of(
            member(doc, "wheel_inertia_cases", "doc"), "doc");
    for (const Value &c: wheels) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const double wm = tag == "front"       ? 2.40
                          : tag == "rear"      ? 2.80
                                               : 1.5;
        const double outer = tag == "front"       ? 0.372
                             : tag == "rear"      ? 0.352
                                                  : 0.30;
        const double hub = tag == "custom_hub" ? 0.06 : 0.045;
        expect_scalar_bits(member(c, "i_yy", "wheel"),
                           specs.compute_wheel_rotational_inertia(
                                   wm, outer, hub),
                           "wheel i_yy " + tag);
    }
}

// ---- model/component_centers + static_system_cg -------------------------------

void test_component_centers() {
    const Value doc = doc_of("model/component_centers");
    const geo::BikeSpecs specs;
    const mdl::BikeMassSpecs mass_specs;
    expect_centers(
            member(doc, "bike_only", "doc"),
            mdl::compute_component_centers_of_mass(specs, mass_specs),
            "bike_only");
    const mdl::RiderSpecs rider{};
    expect_centers(
            member(doc, "with_seated_rider", "doc"),
            mdl::compute_component_centers_of_mass(specs, mass_specs,
                                                   &rider),
            "with_seated_rider");
    // mass.py's saddle_top_z_for_rider rides the same pose: only the
    // seated variant moves the saddle.
    const mdl::SeatedPose pose = mdl::solve_seated_pose(specs, rider);
    const std::optional<double> top =
            mdl::saddle_top_z_for_rider(specs, &rider);
    bt::require(top.has_value() &&
                    same_bits(*top, pose.saddle.top_z_m),
                "saddle_top_z_for_rider seated");
    mdl::RiderSpecs lumped;
    lumped.variant = "lumped";
    cfg::detail::post_init(lumped);
    bt::require(!mdl::saddle_top_z_for_rider(specs, &lumped).has_value() &&
                    !mdl::saddle_top_z_for_rider(specs, nullptr)
                             .has_value(),
                "saddle_top_z_for_rider non-seated/None");
}

void test_static_system_cg() {
    const Value doc = doc_of("model/static_system_cg");
    const geo::BikeSpecs specs;
    const mdl::BikeMassSpecs mass_specs;
    const mdl::RiderSpecs rider{};

    const auto check = [](const Value &sect, const mdl::StaticSystemCg &g,
                          std::string_view label) {
        expect_scalar_bits(member(sect, "total_mass_kg", label),
                           g.total_mass_kg, label);
        expect_scalar_bits(member(sect, "bike_mass_kg", label),
                           g.bike_mass_kg, label);
        expect_scalar_bits(member(sect, "rider_mass_kg", label),
                           g.rider_mass_kg, label);
        expect_bits(member(sect, "cg_pos_m", label), g.cg_pos_m, label);
        expect_scalar_bits(member(sect, "wheelbase_m", label),
                           g.wheelbase_m, label);
        expect_scalar_bits(member(sect, "front_axle_x", label),
                           g.front_axle_x, label);
        expect_scalar_bits(member(sect, "rear_axle_x", label),
                           g.rear_axle_x, label);
        expect_scalar_bits(member(sect, "front_load_n", label),
                           g.front_load_n, label);
        expect_scalar_bits(member(sect, "rear_load_n", label),
                           g.rear_load_n, label);
        expect_scalar_bits(member(sect, "front_load_pct", label),
                           g.front_load_pct, label);
        expect_scalar_bits(member(sect, "rear_load_pct", label),
                           g.rear_load_pct, label);
    };
    check(member(doc, "bike_only", "doc"),
          mdl::compute_static_system_cg(specs, mass_specs), "bike_only");
    check(member(doc, "with_seated_rider", "doc"),
          mdl::compute_static_system_cg(specs, mass_specs, &rider),
          "with_seated_rider");
}

// ---- model/rider_specs ---------------------------------------------------------

void expect_rider_record(const Value &rec, const mdl::RiderSpecs &r,
                         std::string_view label) {
    bt::require(string_of(member(rec, "variant", label), label) ==
                        r.variant,
                std::string(label) + ": variant");
    bt::require(string_of(member(rec, "legs", label), label) == r.legs,
                std::string(label) + ": legs");
    expect_scalar_bits(member(rec, "mass_kg", label), r.mass_kg,
                       std::string(label) + ": mass_kg");
    expect_scalar_bits(member(rec, "height_m", label), r.height_m,
                       std::string(label) + ": height_m");
    const Value &inseam = member(rec, "inseam_m", label);
    bt::require(io::is_null(inseam) == !r.inseam_m.has_value(),
                std::string(label) + ": inseam_m presence");
    if (r.inseam_m.has_value())
        expect_scalar_bits(inseam, *r.inseam_m, std::string(label) + ": inseam_m");
    expect_scalar_bits(member(rec, "helmet_mass_kg", label),
                       r.helmet_mass_kg, std::string(label) + ": helmet");
    expect_scalar_bits(member(rec, "saddle_share", label),
                       r.saddle_share, std::string(label) + ": saddle_share");
    expect_scalar_bits(member(rec, "pedal_share", label),
                       r.pedal_share, std::string(label) + ": pedal_share");
    expect_scalar_bits(member(rec, "bar_share", label), r.bar_share,
                       std::string(label) + ": bar_share");
    expect_scalar_bits(member(rec, "torso_resonance_hz", label),
                       r.torso_resonance_hz, std::string(label) + ": torso_hz");
    expect_scalar_bits(member(rec, "leg_resonance_hz", label),
                       r.leg_resonance_hz, std::string(label) + ": leg_hz");
    expect_scalar_bits(member(rec, "arm_resonance_hz", label),
                       r.arm_resonance_hz, std::string(label) + ": arm_hz");
    expect_scalar_bits(member(rec, "damping_ratio", label),
                       r.damping_ratio, std::string(label) + ": damping");
    expect_scalar_bits(member(rec, "saddle_interface_k_n_m", label),
                       r.saddle_interface_k_n_m, std::string(label) + ": saddle_k");
    expect_scalar_bits(member(rec, "saddle_interface_c_ns_m", label),
                       r.saddle_interface_c_ns_m, std::string(label) + ": saddle_c");
}

void test_rider_specs() {
    const Value doc = doc_of("model/rider_specs");

    const Value::Array &cases =
            array_of(member(doc, "resolve_cases", "doc"), "doc");
    for (const Value &c: cases) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const Value &rider_v = member(c, "rider", "c");
        const Value &inc_v = member(c, "include_rider", "c");
        const std::optional<bool> include =
                io::is_null(inc_v) ? std::nullopt
                                   : std::optional<bool>(
                                             bool_of(inc_v, "c"));
        const std::string &def =
                string_of(member(c, "default_variant", "c"), "c");
        if (tag == "wrong_type") {
            // Python rejects non-(RiderSpecs|str|None) inputs with a
            // TypeError; the C++ RiderArg variant cannot carry them —
            // the branch is eliminated by the type system.
            bt::require(!outcome_accepted(c, "resolve " + tag) &&
                            string_of(member(c, "error", "c"), "c") ==
                                    "TypeError",
                        "wrong_type must stay a TypeError row");
            continue;
        }
        mdl::RiderArg arg = std::monostate{};
        if (tag == "specs_passthrough") {
            mdl::RiderSpecs pass;
            pass.variant = "lumped";
            pass.mass_kg = 70.0;
            cfg::detail::post_init(pass);
            arg = pass;
        } else if (!io::is_null(rider_v)) {
            arg = string_of(rider_v, "rider");
        }
        if (outcome_accepted(c, "resolve " + tag)) {
            const mdl::RiderSpecs got =
                    mdl::resolve_rider(arg, include, def);
            expect_rider_record(*outcome_result(c, "resolve " + tag),
                                got, "resolve " + tag);
        } else {
            expect_invalid_msg(
                    outcome_message(c, "resolve " + tag),
                    [&] { (void)mdl::resolve_rider(arg, include, def); },
                    "resolve " + tag);
        }
    }

    expect_rider_record(member(doc, "rider_from_args_75_175", "doc"),
                        mdl::rider_from_args("seated", 75.0, 1.75,
                                             std::nullopt),
                        "rider_from_args");
    const mdl::RiderSpecs def_rider{};
    expect_rider_record(member(doc, "default_specs", "doc"), def_rider,
                        "default_specs");

    const Value &der = member(doc, "default_derived", "doc");
    bt::require(bool_of(member(der, "present", "der"), "der") ==
                    def_rider.present(),
                "derived present");
    expect_scalar_bits(member(der, "total_rider_mass", "der"),
                       def_rider.total_rider_mass(), "derived mass");
    expect_scalar_bits(member(der, "inseam", "der"), def_rider.inseam(),
                       "derived inseam");
    expect_scalar_bits(member(der, "saddle_height_m", "der"),
                       def_rider.saddle_height_m(), "derived saddle");
    expect_scalar_bits(member(der, "torso_helmet_mass", "der"),
                       def_rider.torso_helmet_mass(), "derived torso");
    expect_scalar_bits(member(der, "legs_mass", "der"),
                       def_rider.legs_mass(), "derived legs");
    expect_scalar_bits(member(der, "arms_mass", "der"),
                       def_rider.arms_mass(), "derived arms");
    const Value &segs = member(der, "segment_lengths", "der");
    for (const std::string_view name:
             {"trunk", "upper_arm", "forearm", "hand", "thigh", "shank",
              "head"})
        expect_scalar_bits(member(segs, name, "segs"),
                           def_rider.segment_length(name),
                           "segment_length " + std::string(name));

    const Value::Array &errs =
            array_of(member(doc, "spec_errors", "doc"), "doc");
    for (const Value &c: errs) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const auto action = [&]() {
            mdl::RiderSpecs s;
            if (tag == "bad_legs") s.legs = "spring";
            else if (tag == "zero_mass") s.mass_kg = 0.0;
            else if (tag == "neg_height") s.height_m = -1.0;
            else if (tag == "inseam_over_height") s.inseam_m = 1.9;
            else if (tag == "shares_not_one") s.saddle_share = 0.5;
            else if (tag == "helmet_ge_mass") s.helmet_mass_kg = 90.0;
            else bt::require(false, "spec error tag: " + tag);
            cfg::detail::post_init(s);
        };
        bt::require(!outcome_accepted(c, "spec " + tag),
                    "spec error must reject: " + tag);
        expect_invalid_msg(outcome_message(c, "spec " + tag), action,
                           "spec " + tag);
    }
}

// ---- model/seated_pose -------------------------------------------------------

// {"$dataclass": "...", "fields": {...}} wrapper — the canonical
// encoder tags nested dataclass objects this way; plain dicts pass
// through unchanged.
const Value &dataclass_fields(const Value &v, std::string_view label) {
    if (std::get_if<Value::Object>(&v.data) != nullptr &&
        member_opt(v, "$dataclass", label) != nullptr) {
        // The recorded class name documents which frozen record the
        // codec observed; the replay only needs the fields payload.
        return member(v, "fields", label);
    }
    // The codec's passthrough returns its input unchanged; callers
    // always hold the parsed doc alive for the replay's duration.
    // NOLINTNEXTLINE(bugprone-return-const-ref-from-parameter)
    return v;
}

void expect_saddle(const Value &sect_v, const geo::SaddleGeometry &g,
                   std::string_view label) {
    const Value &sect = dataclass_fields(sect_v, label);
    expect_bits(member(sect, "seatpost_top", label), g.seatpost_top,
                std::string(label) + ".seatpost_top");
    expect_bits(member(sect, "saddle_pos", label), g.saddle_pos,
                std::string(label) + ".saddle_pos");
    expect_bits(member(sect, "top_center", label), g.top_center,
                std::string(label) + ".top_center");
    expect_scalar_bits(member(sect, "top_z_m", label), g.top_z_m,
                       std::string(label) + ".top_z_m");
    expect_scalar_bits(member(sect, "height_m", label), g.height_m,
                       std::string(label) + ".height_m");
    expect_scalar_bits(member(sect, "exposed_post_m", label),
                       g.exposed_post_m, std::string(label) + ".exposed_post_m");
}

void expect_body(const Value &sect, const mdl::RiderBody &b,
                 std::string_view label) {
    bt::require(string_of(member(sect, "name", label), label) == b.name,
                std::string(label) + ".name");
    bt::require(string_of(member(sect, "parent", label), label) ==
                        b.parent,
                std::string(label) + ".parent");
    bt::require(string_of(member(sect, "joint", label), label) ==
                        b.joint,
                std::string(label) + ".joint");
    expect_bits(member(sect, "attach", label), b.attach,
                std::string(label) + ".attach");
    const Value::Array &geoms =
            array_of(member(sect, "geoms", label), label);
    bt::require(geoms.size() == b.geoms.size(), std::string(label) + ".geom count");
    for (std::size_t i = 0; i < geoms.size(); ++i) {
        const Value &gv = geoms.at(i);
        const mdl::RiderGeom &g = b.geoms.at(i);
        bt::require(string_of(member(gv, "name", label), label) ==
                            g.name &&
                            string_of(member(gv, "kind", label), label) ==
                                    g.kind,
                    std::string(label) + ".geom " + std::to_string(i));
        expect_bits(member(gv, "p1", label), g.p1, std::string(label) + ".p1");
        expect_bits(member(gv, "p2", label), g.p2, std::string(label) + ".p2");
        expect_scalar_bits(member(gv, "radius", label), g.radius,
                           std::string(label) + ".radius");
        expect_scalar_bits(member(gv, "mass", label), g.mass,
                           std::string(label) + ".geom mass");
    }
    expect_scalar_bits(member(sect, "stiffness_n_m", label),
                       b.stiffness_n_m, std::string(label) + ".stiffness");
    expect_scalar_bits(member(sect, "damping_ns_m", label),
                       b.damping_ns_m, std::string(label) + ".damping");
    expect_scalar_bits(member(sect, "supported_mass_kg", label),
                       b.supported_mass_kg, std::string(label) + ".supported");
    bt::require(bool_of(member(sect, "unilateral", label), label) ==
                        b.unilateral,
                std::string(label) + ".unilateral");
    bt::require(string_of(member(sect, "interface", label), label) ==
                        b.interface,
                std::string(label) + ".interface");
    expect_scalar_bits(member(sect, "mass", label), b.mass(),
                       std::string(label) + ".mass");
    expect_bits(member(sect, "center_of_mass", label),
                b.center_of_mass(), std::string(label) + ".center_of_mass");
    expect_scalar_bits(member(sect, "preload_n", label), b.preload_n(),
                       std::string(label) + ".preload_n");
    expect_scalar_bits(member(sect, "preload_deflection_m", label),
                       b.preload_deflection_m(),
                       std::string(label) + ".preload_deflection_m");
}

void expect_chain(const Value &sect, const mdl::LegChain &c,
                  std::string_view label) {
    bt::require(string_of(member(sect, "side", label), label) == c.side,
                std::string(label) + ".side");
    expect_scalar_bits(member(sect, "lateral_y_m", label),
                       c.lateral_y_m, std::string(label) + ".lateral_y");
    expect_scalar_bits(member(sect, "crank_len_m", label), c.crank_len_m,
                       std::string(label) + ".crank_len");
    expect_bits(member(sect, "hip", label), c.hip, std::string(label) + ".hip");
    expect_bits(member(sect, "knee", label), c.knee, std::string(label) + ".knee");
    expect_bits(member(sect, "ankle", label), c.ankle, std::string(label) + ".ankle");
    expect_bits(member(sect, "pedal", label), c.pedal, std::string(label) + ".pedal");
    expect_scalar_bits(member(sect, "thigh_len_m", label), c.thigh_len_m,
                       std::string(label) + ".thigh_len");
    expect_scalar_bits(member(sect, "shank_len_m", label), c.shank_len_m,
                       std::string(label) + ".shank_len");
    expect_scalar_bits(member(sect, "thigh_mass_kg", label),
                       c.thigh_mass_kg, std::string(label) + ".thigh_mass");
    expect_scalar_bits(member(sect, "shank_mass_kg", label),
                       c.shank_mass_kg, std::string(label) + ".shank_mass");
    expect_scalar_bits(member(sect, "foot_mass_kg", label),
                       c.foot_mass_kg, std::string(label) + ".foot_mass");
    expect_scalar_bits(member(sect, "pitch0_thigh_rad", label),
                       c.pitch0_thigh_rad, std::string(label) + ".pitch0_thigh");
    expect_scalar_bits(member(sect, "pitch0_shank_rad", label),
                       c.pitch0_shank_rad, std::string(label) + ".pitch0_shank");
    expect_scalar_bits(member(sect, "pitch0_foot_rad", label),
                       c.pitch0_foot_rad, std::string(label) + ".pitch0_foot");
}

void expect_pose(const Value &sect, const mdl::SeatedPose &pose,
                 std::string_view label) {
    expect_saddle(member(sect, "saddle", label), pose.saddle,
                  std::string(label) + ".saddle");
    for (const std::string_view key:
             {"hip", "shoulder", "elbow", "wrist", "grip", "head_center",
              "knee_front", "knee_rear", "ankle_front", "ankle_rear",
              "pedal_front", "pedal_rear"}) {
        const geo::Vec3 &p =
                key == "hip"          ? pose.hip
                : key == "shoulder"   ? pose.shoulder
                : key == "elbow"      ? pose.elbow
                : key == "wrist"      ? pose.wrist
                : key == "grip"       ? pose.grip
                : key == "head_center" ? pose.head_center
                : key == "knee_front" ? pose.knee_front
                : key == "knee_rear"  ? pose.knee_rear
                : key == "ankle_front" ? pose.ankle_front
                : key == "ankle_rear" ? pose.ankle_rear
                : key == "pedal_front" ? pose.pedal_front
                                       : pose.pedal_rear;
        expect_bits(member(sect, key, label), p,
                    std::string(label) + "." + std::string(key));
    }
    expect_scalar_bits(member(sect, "torso_lean_deg", label),
                       pose.torso_lean_deg, std::string(label) + ".torso_lean");
    expect_scalar_bits(member(sect, "knee_flexion_front_deg", label),
                       pose.knee_flexion_front_deg,
                       std::string(label) + ".flexion_front");
    expect_scalar_bits(member(sect, "knee_flexion_rear_deg", label),
                       pose.knee_flexion_rear_deg,
                       std::string(label) + ".flexion_rear");
    expect_scalar_bits(member(sect, "knee_flexion_bdc_deg", label),
                       pose.knee_flexion_bdc_deg, std::string(label) + ".flexion_bdc");
    const Value::Array &bodies =
            array_of(member(sect, "bodies", label), label);
    bt::require(bodies.size() == pose.bodies.size(),
                std::string(label) + ".body count");
    for (std::size_t i = 0; i < bodies.size(); ++i)
        expect_body(bodies.at(i), pose.bodies.at(i),
                    std::string(label) + ".bodies[" + std::to_string(i) + "]");
    expect_pairs(member(sect, "path_masses_kg", label),
                 pose.path_masses_kg, std::string(label) + ".path_masses");
    const Value::Array &chains =
            array_of(member(sect, "leg_chains", label), label);
    bt::require(chains.size() == pose.leg_chains.size(),
                std::string(label) + ".chain count");
    for (std::size_t i = 0; i < chains.size(); ++i)
        expect_chain(chains.at(i), pose.leg_chains.at(i),
                     std::string(label) + ".chain[" + std::to_string(i) + "]");
    expect_scalar_bits(member(sect, "total_mass_kg", label),
                       pose.total_mass_kg(), std::string(label) + ".total_mass");
    expect_pairs(member(sect, "interface_loads_n", label),
                 pose.interface_loads_n(), std::string(label) + ".loads");
    expect_pairs(member(sect, "interface_shares", label),
                 pose.interface_shares(), std::string(label) + ".shares");
    const Value &miss = member(sect, "missing_body_error", label);
    bt::require(!outcome_accepted(miss, std::string(label) + ".missing"),
                "missing body must reject");
    bt::require(string_of(member(miss, "error", label), label) ==
                        "KeyError",
                std::string(label) + ".missing error kind");
    expect_key_error(outcome_message(miss, std::string(label) + ".missing"),
                     [&] { (void)pose.body("rider_nothing"); },
                     std::string(label) + ".missing_body_error");
}

void test_seated_pose() {
    const Value doc = doc_of("model/seated_pose");
    const geo::BikeSpecs specs;

    mdl::RiderSpecs rider_75;
    rider_75.mass_kg = 75.0;
    rider_75.height_m = 1.75;
    cfg::detail::post_init(rider_75);

    mdl::RiderSpecs artic;
    artic.legs = "articulated";
    artic.mass_kg = 75.0;
    artic.height_m = 1.75;
    cfg::detail::post_init(artic);

    expect_pose(member(doc, "default", "doc"),
                mdl::solve_seated_pose(specs, mdl::RiderSpecs{}),
                "default");
    expect_pose(member(doc, "rider_75_175", "doc"),
                mdl::solve_seated_pose(specs, rider_75), "rider_75_175");
    expect_pose(member(doc, "legs_articulated", "doc"),
                mdl::solve_seated_pose(specs, artic),
                "legs_articulated");
}

// ---- model/rider_helpers -------------------------------------------------------

void test_rider_helpers() {
    const Value doc = doc_of("model/rider_helpers");

    const Value::Array &iks =
            array_of(member(doc, "ik_cases", "doc"), "doc");
    for (const Value &c: iks) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const geo::Vec3 a = vec_of(member(c, "a", "c"), "ik " + tag);
        const geo::Vec3 b = vec_of(member(c, "b", "c"), "ik " + tag);
        const double la =
                scalar_field(member(c, "len_a", "c"), "ik " + tag);
        const double lb =
                scalar_field(member(c, "len_b", "c"), "ik " + tag);
        const std::string &prefer =
                string_of(member(c, "prefer", "c"), "c");
        if (outcome_accepted(c, "ik " + tag)) {
            expect_bits(*outcome_result(c, "ik " + tag),
                        mdl::two_link_ik(a, b, la, lb, prefer),
                        "ik " + tag);
        } else if (tag == "bad_prefer") {
            // dict KeyError for an unknown prefer branch.
            expect_key_error(
                    outcome_message(c, "ik " + tag),
                    [&] { (void)mdl::two_link_ik(a, b, la, lb, prefer); },
                    "ik " + tag);
        } else {
            expect_invalid_msg(
                    outcome_message(c, "ik " + tag),
                    [&] { (void)mdl::two_link_ik(a, b, la, lb, prefer); },
                    "ik " + tag);
        }
    }

    const Value::Array &flex =
            array_of(member(doc, "flexion_cases", "doc"), "doc");
    for (const Value &c: flex) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const geo::Vec3 p = vec_of(member(c, "proximal", "c"), "fx");
        const geo::Vec3 j = vec_of(member(c, "joint", "c"), "fx");
        const geo::Vec3 d = vec_of(member(c, "distal", "c"), "fx");
        bt::require(outcome_accepted(c, "flex " + tag),
                    "flexion case must accept: " + tag);
        expect_scalar_bits(*outcome_result(c, "flex " + tag),
                           mdl::flexion_deg(p, j, d), "flex " + tag);
    }

    const Value::Array &springs =
            array_of(member(doc, "spring_cases", "doc"), "doc");
    for (const Value &c: springs) {
        bt::require(outcome_accepted(c, "spring"), "spring accepts");
        const auto [k, d] = mdl::spring_params(30.0, 5.0, 0.4);
        const std::vector<double> got{k, d};
        const std::vector<double> want =
                scalar_list(*outcome_result(c, "spring"), "spring");
        bt::require(got.size() == want.size(), "spring arity");
        for (std::size_t i = 0; i < got.size(); ++i)
            bt::require(same_bits(got.at(i), want.at(i)),
                        "spring lane " + std::to_string(i));
    }

    const Value::Array &paths =
            array_of(member(doc, "path_cases", "doc"), "doc");
    for (const Value &c: paths) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const auto make = [&]() -> mdl::RiderSpecs {
            mdl::RiderSpecs r;
            if (tag == "rider_75") {
                r.mass_kg = 75.0;
                r.height_m = 1.75;
            } else if (tag == "bar_too_low") {
                r.bar_share = 0.05;
                r.pedal_share = 0.40;
            } else if (tag == "pedal_too_low") {
                r.pedal_share = 0.05;
                r.bar_share = 0.40;
            } else if (tag == "pedal_too_high") {
                r.pedal_share = 0.80;
                r.saddle_share = 0.10;
                r.bar_share = 0.10;
            } else if (tag == "helmet_heavy") {
                r.mass_kg = 60.0;
                r.helmet_mass_kg = 5.0;
                r.saddle_share = 0.40;
                r.pedal_share = 0.30;
                r.bar_share = 0.30;
            }
            cfg::detail::post_init(r);
            return r;
        };
        if (outcome_accepted(c, "path " + tag)) {
            const mdl::RiderSpecs r = make();
            expect_pairs(*outcome_result(c, "path " + tag),
                         mdl::seated_path_masses(r), "path " + tag);
        } else {
            expect_invalid_msg(
                    outcome_message(c, "path " + tag),
                    [&] {
                        const mdl::RiderSpecs r = make();
                        (void)mdl::seated_path_masses(r);
                    },
                    "path " + tag);
        }
    }
}

// ---- model/rider_legs_centers --------------------------------------------------

void test_rider_legs_centers() {
    const Value doc = doc_of("model/rider_legs_centers");
    const geo::BikeSpecs specs;

    mdl::RiderSpecs artic;
    artic.legs = "articulated";
    artic.mass_kg = 75.0;
    artic.height_m = 1.75;
    cfg::detail::post_init(artic);
    const mdl::SeatedPose pose_leg =
            mdl::solve_seated_pose(specs, artic);
    bt::require(pose_leg.leg_chains.size() == 2,
                "articulated pose needs two leg chains");

    const Value::Array &legs =
            array_of(member(doc, "leg_cases", "doc"), "doc");
    for (const Value &c: legs) {
        const std::string &side =
                string_of(member(c, "side", "c"), "c");
        const double phase =
                scalar_field(member(c, "phase", "c"), "c");
        const mdl::LegChain &chain =
                side == "front" ? pose_leg.leg_chains.at(0)
                                : pose_leg.leg_chains.at(1);
        bt::require(outcome_accepted(c, "leg " + side),
                    "leg case must accept: " + side);
        const Value &res = *outcome_result(c, "leg " + side);
        const geo::Vec3 qpos = mdl::solve_leg_qpos(chain, phase, 0.0);
        expect_bits(member(res, "qpos", "leg"), qpos, "leg.qpos");
        if (const Value *k_ik = member_opt(res, "knee_ik", "leg")) {
            const auto [knee_ik, ankle_ik] =
                    mdl::solve_leg_joints(chain, phase, 0.0);
            expect_bits(*k_ik, knee_ik, "leg.knee_ik");
            expect_bits(member(res, "ankle_ik", "leg"), ankle_ik,
                        "leg.ankle_ik");
            const mdl::LegFkResult fk = mdl::fk_leg(chain, qpos, 0.0);
            expect_bits(member(res, "knee_fk", "leg"), fk.knee,
                        "leg.knee_fk");
            expect_bits(member(res, "ankle_fk", "leg"), fk.ankle,
                        "leg.ankle_fk");
            expect_bits(member(res, "pedal_fk", "leg"), fk.pedal_pt,
                        "leg.pedal_fk");
            const mdl::Mat32 jac = mdl::leg_jacobian(chain, qpos, 0.0);
            const std::vector<double> flat{
                    jac[0][0], jac[0][1], jac[1][0],
                    jac[1][1], jac[2][0], jac[2][1]};
            expect_bits(member(res, "jacobian", "leg"), flat,
                        "leg.jacobian");
            expect_scalar_bits(member(res, "crank_torque_share", "leg"),
                               mdl::crank_torque_share(phase, 0.6),
                               "leg.crank_torque_share");
            expect_bits(member(res, "pedal_spindle", "leg"),
                        mdl::pedal_spindle_pos(phase, chain.lateral_y_m,
                                               chain.crank_len_m),
                        "leg.pedal_spindle");
        }
    }

    const Value &variants = member(doc, "center_variants", "doc");
    const auto rider_for = [](std::string_view tag) {
        mdl::RiderSpecs r;
        if (tag == "none") r.variant = "none";
        else if (tag == "lumped") r.variant = "lumped";
        else if (tag == "seated_articulated_legs") {
            r.legs = "articulated";
            r.mass_kg = 75.0;
            r.height_m = 1.75;
        } else if (tag == "articulated_planar")
            r.variant = "articulated_planar";
        cfg::detail::post_init(r);
        return r;
    };
    for (const std::string_view tag:
             {"none", "lumped", "seated", "seated_articulated_legs",
              "articulated_planar"}) {
        const Value &v = member(variants, tag, "variants");
        const mdl::RiderSpecs r = rider_for(tag);
        bt::require(outcome_accepted(v, "variant " + std::string(tag)),
                    "center variant must accept: " + std::string(tag));
        expect_centers(*outcome_result(v, "variant " + std::string(tag)),
                       mdl::compute_rider_centers_of_mass(r, specs),
                       "variant " + std::string(tag));
    }

    const mdl::SeatedPose pose_default =
            mdl::solve_seated_pose(specs, mdl::RiderSpecs{});
    const std::vector<double> apparent = mdl::saddle_path_apparent_mass(
            pose_default, std::vector<double>{0.5, 1.0, 2.0, 4.8, 8.0,
                                              20.0});
    expect_bits(member(doc, "apparent_mass", "doc"), apparent,
                "apparent_mass");
}

// ---- model/rider_segments --------------------------------------------------------

void test_rider_segments() {
    const Value doc = doc_of("model/rider_segments");

    const PairExpectation de_leva = pairs_of(
            member(doc, "de_leva_fractions", "doc"), "de_leva");
    for (std::size_t i = 0; i < de_leva.names.size(); ++i) {
        bt::require(mdl::has_de_leva_mass_fraction(de_leva.names.at(i)),
                    "de_leva key missing: " + de_leva.names.at(i));
        bt::require(same_bits(
                            mdl::de_leva_mass_fraction(de_leva.names.at(i)),
                            de_leva.values.at(i)),
                    "de_leva bits: " + de_leva.names.at(i));
    }
    bt::require(!mdl::has_de_leva_mass_fraction("nothing"),
                "de_leva miss probe");

    expect_scalar_bits(member(doc, "arm_lateral_offset_m", "doc"),
                       mdl::ARM_LATERAL_OFFSET_M, "arm offset");

    const Value::Array &mass_cases =
            array_of(member(doc, "mass_cases", "doc"), "doc");
    for (const Value &c: mass_cases) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const double total =
                scalar_field(member(c, "total_kg", "c"), "c");
        const double helmet =
                scalar_field(member(c, "helmet_kg", "c"), "c");
        if (outcome_accepted(c, "segmass " + tag)) {
            expect_pairs(*outcome_result(c, "segmass " + tag),
                         mdl::segment_masses(total, helmet),
                         "segmass " + tag);
        } else {
            expect_invalid_msg(
                    outcome_message(c, "segmass " + tag),
                    [&] { (void)mdl::segment_masses(total, helmet); },
                    "segmass " + tag);
        }
    }

    expect_pairs(member(doc, "split_pairs", "doc"),
                 mdl::split_paired_arm_masses(
                         mdl::segment_masses(75.0, 0.4)),
                 "split_pairs");

    const Value::Array &rows =
            array_of(member(doc, "inertia_rows", "doc"), "doc");
    for (const Value &c: rows) {
        const std::string &tag = string_of(member(c, "name", "c"), "c");
        const double mass =
                scalar_field(member(c, "mass_kg", "c"), "c");
        const geo::Vec3 axis =
                vec_of(member(c, "axis_vector_m", "c"), "c");
        const double radius =
                scalar_field(member(c, "radius_m", "c"), "c");
        bt::require(outcome_accepted(c, "seg inertia " + tag),
                    "segment inertia row must accept: " + tag);
        expect_bits(*outcome_result(c, "seg inertia " + tag),
                    mdl::segment_inertia(mass, axis, radius),
                    "seg inertia " + tag);
    }

    const Value::Array &errs = array_of(
            member(doc, "inertia_error_cases", "doc"), "doc");
    for (const Value &c: errs) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const double mass = tag == "zero_mass" ? 0.0 : 1.0;
        geo::Vec3 axis{0.3, 0.0, 0.3};
        double radius = 0.05;
        if (tag == "zero_axis") axis = {0.0, 0.0, 0.0};
        else if (tag == "neg_radius") radius = -0.05;
        else if (tag == "nan_axis")
            axis = {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
        bt::require(!outcome_accepted(c, "seg err " + tag),
                    "segment error must reject: " + tag);
        expect_invalid_msg(
                outcome_message(c, "seg err " + tag),
                [&] { (void)mdl::segment_inertia(mass, axis, radius); },
                "seg err " + tag);
    }

    mdl::RiderSpecs artic_variant;
    artic_variant.variant = "articulated_planar";
    cfg::detail::post_init(artic_variant);
    const mdl::ArticulatedPose pose =
            mdl::geometry_pose(artic_variant, geo::BikeSpecs{});
    const Value &ap = member(doc, "articulated_pose", "doc");
    expect_saddle(member(ap, "saddle", "ap"), pose.saddle, "ap.saddle");
    for (const std::string_view key:
             {"hip", "shoulder", "elbow", "grip", "head_center",
              "knee_front", "knee_rear", "ankle_front", "ankle_rear",
              "pedal_front", "pedal_rear"}) {
        const geo::Vec3 &p =
                key == "hip"          ? pose.hip
                : key == "shoulder"   ? pose.shoulder
                : key == "elbow"      ? pose.elbow
                : key == "grip"       ? pose.grip
                : key == "head_center" ? pose.head_center
                : key == "knee_front" ? pose.knee_front
                : key == "knee_rear"  ? pose.knee_rear
                : key == "ankle_front" ? pose.ankle_front
                : key == "ankle_rear" ? pose.ankle_rear
                : key == "pedal_front" ? pose.pedal_front
                                       : pose.pedal_rear;
        expect_bits(member(ap, key, "ap"), p,
                    "ap." + std::string(key));
    }
}

// ---- model/air_spring -----------------------------------------------------------

void test_air_spring() {
    const Value doc = doc_of("model/air_spring");
    const mdl::AirSpringSpecs specs;
    const Value &sd = member(doc, "specs", "doc");
    expect_scalar_bits(member(sd, "stanchion_inner_diam_mm", "sd"),
                       specs.stanchion_inner_diam_mm, "spec diam");
    expect_scalar_bits(member(sd, "total_travel_mm", "sd"),
                       specs.total_travel_mm, "spec travel");
    expect_scalar_bits(member(sd, "pos_chamber_length_mm", "sd"),
                       specs.pos_chamber_length_mm, "spec pos_len");
    expect_scalar_bits(member(sd, "neg_chamber_length_mm", "sd"),
                       specs.neg_chamber_length_mm, "spec neg_len");
    expect_scalar_bits(member(sd, "token_volume_cm3", "sd"),
                       specs.token_volume_cm3, "spec token_vol");
    bt::require(static_cast<int>(
                        number_of(member(sd, "max_tokens", "sd"))) ==
                        specs.max_tokens,
                "spec max_tokens");
    bt::require(static_cast<int>(number_of(
                        member(sd, "default_tokens", "sd"))) ==
                        specs.default_tokens,
                "spec default_tokens");
    expect_scalar_bits(member(sd, "gamma", "sd"), specs.gamma,
                       "spec gamma");
    expect_scalar_bits(member(sd, "atm_pressure_pa", "sd"),
                       specs.atm_pressure_pa, "spec atm");

    const Value &props = member(doc, "properties", "doc");
    expect_scalar_bits(member(props, "piston_area_m2", "p"),
                       specs.piston_area_m2(), "piston_area");
    expect_scalar_bits(member(props, "token_volume_m3", "p"),
                       specs.token_volume_m3(), "token_volume");
    expect_scalar_bits(member(props, "base_pos_volume_m3", "p"),
                       specs.base_pos_volume_m3(), "base_pos");
    expect_scalar_bits(member(props, "base_neg_volume_m3", "p"),
                       specs.base_neg_volume_m3(), "base_neg");

    const mdl::ForkAirSpring spring;
    const Value &defs = member(doc, "defaults", "doc");
    bt::require(static_cast<int>(
                        number_of(member(defs, "num_tokens", "d"))) ==
                        spring.num_tokens(),
                "default tokens");
    expect_scalar_bits(member(defs, "gauge_pressure_psi", "d"),
                       spring.gauge_pressure_psi(), "default psi");
    expect_scalar_bits(member(defs, "abs_pressure_pa", "d"),
                       spring.abs_pressure_pa(), "abs_pressure");
    expect_scalar_bits(member(defs, "gauge_pressure_bar", "d"),
                       spring.gauge_pressure_bar(), "gauge_bar");

    const Value::Array &vols =
            array_of(member(doc, "volume_rows", "doc"), "doc");
    for (const Value &c: vols) {
        const double travel =
                scalar_field(member(c, "travel_mm", "c"), "c");
        bt::require(outcome_accepted(c, "vol row"), "vol row accepts");
        const auto [vp, vn] = spring.compute_volumes(travel);
        const std::vector<double> want = scalar_list(
                *outcome_result(c, "vol row"), "vol");
        bt::require(want.size() == 2 && same_bits(vp, want.at(0)) &&
                            same_bits(vn, want.at(1)),
                    "volume row bits " + std::to_string(travel));
    }
    const Value::Array &press =
            array_of(member(doc, "pressure_rows", "doc"), "doc");
    for (const Value &c: press) {
        const double travel =
                scalar_field(member(c, "travel_mm", "c"), "c");
        bt::require(outcome_accepted(c, "press row"),
                    "press row accepts");
        const auto [pp, pn] = spring.compute_pressures_pa(travel);
        const std::vector<double> want = scalar_list(
                *outcome_result(c, "press row"), "press");
        bt::require(want.size() == 2 && same_bits(pp, want.at(0)) &&
                            same_bits(pn, want.at(1)),
                    "pressure row bits " + std::to_string(travel));
    }
    const Value::Array &forces =
            array_of(member(doc, "force_rows", "doc"), "doc");
    for (const Value &c: forces) {
        const double travel =
                scalar_field(member(c, "travel_mm", "c"), "c");
        bt::require(outcome_accepted(c, "force row"),
                    "force row accepts");
        expect_scalar_bits(*outcome_result(c, "force row"),
                           spring.compute_axial_force(travel),
                           "force row");
    }
    const Value::Array &stiffs =
            array_of(member(doc, "stiffness_rows", "doc"), "doc");
    for (const Value &c: stiffs) {
        const double travel =
                scalar_field(member(c, "travel_mm", "c"), "c");
        bt::require(outcome_accepted(c, "stiff row"),
                    "stiff row accepts");
        expect_scalar_bits(*outcome_result(c, "stiff row"),
                           spring.compute_instantaneous_stiffness(travel),
                           "stiff row");
    }

    const Value &curve = member(doc, "force_curve", "doc");
    const mdl::AirSpringForceCurve got =
            spring.compute_force_curve(41);
    for (const std::string_view key:
             {"travel_mm", "axial_force_n", "stiffness_n_mm",
              "pos_pressure_psi", "neg_pressure_psi", "energy_j"}) {
        const std::vector<double> &lane =
                key == "travel_mm"          ? got.travel_mm
                : key == "axial_force_n"    ? got.axial_force_n
                : key == "stiffness_n_mm"   ? got.stiffness_n_mm
                : key == "pos_pressure_psi" ? got.pos_pressure_psi
                : key == "neg_pressure_psi" ? got.neg_pressure_psi
                                            : got.energy_j;
        expect_bits(member(curve, key, "curve"), lane,
                    "curve." + std::string(key));
    }
    expect_scalar_bits(member(curve, "max_force_n", "curve"),
                       got.max_force_n, "curve.max_force");
    expect_scalar_bits(member(curve, "max_energy_j", "curve"),
                       got.max_energy_j, "curve.max_energy");
    expect_scalar_bits(member(curve, "stiffness_at_0_n_mm", "curve"),
                       got.stiffness_at_0_n_mm, "curve.stiff0");
    expect_scalar_bits(member(curve, "stiffness_at_sag_n_mm", "curve"),
                       got.stiffness_at_sag_n_mm, "curve.stiffsag");
    expect_scalar_bits(member(curve, "stiffness_at_bottom_n_mm", "curve"),
                       got.stiffness_at_bottom_n_mm, "curve.stiffbot");
    expect_scalar_bits(member(curve, "progressivity_pct", "curve"),
                       got.progressivity_pct, "curve.progressivity");

    expect_bits(member(doc, "fit_order3", "doc"),
                    spring.fit_polynomial_mjcf(), "fit_order3");

    const Value &cal = member(doc, "calibration", "doc");
    mdl::ForkAirSpring cal_spring;
    const Value &cal_out = member(cal, "outcome", "cal");
    bt::require(outcome_accepted(cal_out, "cal"), "calibration accepts");
    expect_scalar_bits(*outcome_result(cal_out, "cal"),
                       cal_spring.calibrate_psi_for_sag(), "cal psi");
    expect_scalar_bits(member(cal, "psi_after", "cal"),
                       cal_spring.gauge_pressure_psi(), "cal psi_after");

    const Value::Array &errs =
            array_of(member(doc, "error_cases", "doc"), "doc");
    for (const Value &c: errs) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        if (tag == "tokens_over_max") {
            mdl::ForkAirSpring s;
            s.set_tokens(99);
            bt::require(s.num_tokens() == 4, "tokens clamped");
        } else if (tag == "neg_psi_ctor") {
            expect_invalid_msg(
                    outcome_message(c, "air " + tag),
                    [] { const mdl::ForkAirSpring bad(std::nullopt, -5.0); },
                    "air " + tag);
        } else if (tag == "set_pressure_clamps") {
            mdl::ForkAirSpring s;
            s.set_pressure_psi(1.0);
            bt::require(same_bits(s.gauge_pressure_psi(), 10.0),
                        "pressure floor");
        } else if (tag == "bad_travel") {
            expect_invalid_msg(
                    outcome_message(c, "air " + tag),
                    [] {
                        (void)mdl::ForkAirSpring{}.compute_volumes(
                                std::numeric_limits<double>::quiet_NaN());
                    },
                    "air " + tag);
        } else if (tag == "overtravel_volumes") {
            const auto [vp, vn] =
                    mdl::ForkAirSpring{}.compute_volumes(1e6);
            const std::vector<double> want = scalar_list(
                    *outcome_result(c, "air " + tag), "air");
            bt::require(want.size() == 2 && same_bits(vp, want.at(0)) &&
                                same_bits(vn, want.at(1)),
                        "overtravel volumes");
        } else if (tag == "exhausted_volumes") {
            mdl::AirSpringSpecs small;
            small.pos_chamber_length_mm = 60.0;
            expect_invalid_msg(
                    outcome_message(c, "air " + tag),
                    [&] {
                        (void)mdl::ForkAirSpring(small).compute_volumes(
                                180.0);
                    },
                    "air " + tag);
        } else if (tag == "bad_travel_type") {
            // compute_volumes("x") is a Python-typed-domain branch; the
            // C++ signature accepts double so this input is
            // unrepresentable by construction.
            bt::require(!outcome_accepted(c, "air " + tag),
                        "bad_travel_type stays a rejection");
        } else if (tag == "lstsq_bad_order") {
            expect_invalid_msg(
                    outcome_message(c, "air " + tag),
                    [] { (void)mdl::ForkAirSpring{}.fit_polynomial_mjcf(0); },
                    "air " + tag);
        } else if (tag == "calibrate_bad_sag") {
            mdl::ForkAirSpring s;
            const double psi = s.calibrate_psi_for_sag(179.9, 100.0);
            expect_scalar_bits(*outcome_result(c, "air " + tag), psi,
                               "air " + tag);
        } else {
            bt::require(false, "air error tag: " + tag);
        }
    }
}

// ---- model/coil_shock -----------------------------------------------------------

void test_coil_shock() {
    const Value doc = doc_of("model/coil_shock");
    const mdl::CoilShockSpecs specs;
    expect_scalar_bits(member(doc, "bumper_engage_mm", "doc"),
                       specs.bumper_engage_mm(), "bumper_engage");

    const mdl::CoilShock coil;
    const mdl::CoilShock legacy(mdl::CoilShockSpecs{}, true);
    const mdl::CoilShock preload(
            [] {
                mdl::CoilShockSpecs s;
                s.preload_mm = 2.0;
                return s;
            }(),
            false);

    const Value::Array &rows =
            array_of(member(doc, "rows", "doc"), "doc");
    for (const Value &c: rows) {
        const double stroke =
                scalar_field(member(c, "stroke_mm", "c"), "c");
        for (const std::string_view kind:
                 {"spring", "bumper", "axial", "legacy_spring",
                  "preload_spring"}) {
            const Value &o =
                    member(c, kind, "coil row");
            bt::require(outcome_accepted(o, "coil row"),
                        "coil outcome accepts: " + std::string(kind));
            const double got =
                    kind == "spring" ? coil.compute_spring_force(stroke)
                    : kind == "bumper" ? coil.compute_bumper_force(stroke)
                    : kind == "axial" ? coil.compute_axial_force(stroke)
                    : kind == "legacy_spring"
                            ? legacy.compute_spring_force(stroke)
                            : preload.compute_spring_force(stroke);
            expect_scalar_bits(*outcome_result(o, "coil row"), got,
                               "coil " + std::string(kind));
        }
    }

    const Value::Array &errs =
            array_of(member(doc, "error_cases", "doc"), "doc");
    for (const Value &c: errs) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const auto action = [&]() {
            if (tag == "nan_stroke") {
                (void)mdl::CoilShock{}.compute_spring_force(
                        std::numeric_limits<double>::quiet_NaN());
                return;
            }
            mdl::CoilShockSpecs s;
            if (tag == "neg_rate") s.rate_n_m = -1.0;
            else if (tag == "zero_stroke") s.stroke_mm = 0.0;
            else if (tag == "bumper_over_stroke") s.bumper_length_mm = 70.0;
            else if (tag == "neg_preload") s.preload_mm = -1.0;
            else bt::require(false, "coil error tag: " + tag);
            mdl::validate_coil_shock_specs(s);
        };
        bt::require(!outcome_accepted(c, "coil " + tag),
                    "coil case must reject: " + tag);
        expect_invalid_msg(outcome_message(c, "coil " + tag), action,
                           "coil " + tag);
    }
}

// ---- model/dampers ---------------------------------------------------------------

mdl::DamperPreset preset_named(std::string_view name) {
    if (name == "BASE") return mdl::DamperPreset::base;
    if (name == "PLUSH") return mdl::DamperPreset::plush;
    if (name == "ENDURO") return mdl::DamperPreset::enduro;
    if (name == "PARK") return mdl::DamperPreset::park;
    bt::require(name == "CUSTOM",
                "preset name: " + std::string(name));
    return mdl::DamperPreset::custom;
}
std::string_view preset_name(mdl::DamperPreset p) {
    switch (p) {
        case mdl::DamperPreset::base: return "BASE";
        case mdl::DamperPreset::plush: return "PLUSH";
        case mdl::DamperPreset::enduro: return "ENDURO";
        case mdl::DamperPreset::park: return "PARK";
        case mdl::DamperPreset::custom: return "CUSTOM";
    }
    bt::require(false, "preset enum");
    std::abort();
}

void expect_click_config(const Value &rec,
                         const mdl::DamperClickConfig &cfg_v,
                         std::string_view label) {
    const auto ints = [&](const char *k, int actual) {
        bt::require(static_cast<int>(number_of(
                            member(rec, k, label))) == actual,
                    std::string(label) + "." + k);
    };
    ints("fork_hsc", cfg_v.fork_hsc);
    ints("fork_lsc", cfg_v.fork_lsc);
    ints("fork_rebound", cfg_v.fork_rebound);
    ints("shock_hsc", cfg_v.shock_hsc);
    ints("shock_lsc", cfg_v.shock_lsc);
    ints("shock_rebound", cfg_v.shock_rebound);
    ints("shock_hbo", cfg_v.shock_hbo);
    bt::require(bool_of(member(rec, "shock_lockout", label), label) ==
                        cfg_v.shock_lockout,
                std::string(label) + ".shock_lockout");
    bt::require(string_of(member(rec, "preset_name", label), label) ==
                        cfg_v.preset_name,
                std::string(label) + ".preset_name");
}

void test_damper_presets() {
    const Value doc = doc_of("model/dampers");

    const Value::Array &table =
            array_of(member(doc, "preset_table", "doc"), "doc");
    static constexpr std::array<mdl::DamperPreset, 5> kOrder{
            mdl::DamperPreset::base, mdl::DamperPreset::plush,
            mdl::DamperPreset::enduro, mdl::DamperPreset::park,
            mdl::DamperPreset::custom};
    bt::require(table.size() == kOrder.size(), "preset table size");
    for (std::size_t i = 0; i < table.size(); ++i) {
        const Value &row = table.at(i);
        const mdl::DamperPreset p = kOrder.at(i);
        bt::require(string_of(member(row, "preset", "row"), "row") ==
                            preset_name(p),
                    "preset enum order");
        bt::require(string_of(member(row, "label", "row"), "row") ==
                            mdl::damper_preset_label(p),
                    "preset label");
        bt::require(bool_of(member(row, "has_config", "row"), "row") ==
                            mdl::damper_preset_has_config(p),
                    "preset has_config");
        const Value &cfg_v = member(row, "config", "row");
        if (io::is_null(cfg_v)) {
            bt::require(!mdl::damper_preset_has_config(p),
                        "custom preset has no config");
        } else {
            expect_click_config(cfg_v, mdl::damper_preset_config(p),
                                "preset config");
        }
    }
    expect_click_config(member(doc, "click_config_defaults", "doc"),
                        mdl::DamperClickConfig{}, "click defaults");

    expect_pairs(member(doc, "fork_coefficients", "doc"),
                 mdl::Charger3Damper{}.get_effective_coefficients(),
                 "fork coefficients");
    // SuperDeluxe's coefficients append c_hbo/frac_hbo/lockout_firm —
    // the lockout entry records the bool verbatim, so decode the pair
    // list by hand rather than through the numeric pairs_of helper.
    {
        const mdl::SuperDeluxeDamper::Coefficients got =
                mdl::SuperDeluxeDamper{}.get_effective_coefficients();
        const std::vector<std::pair<std::string, double>> flat{
                {"c_lsc", got.c_lsc},        {"c_hsc", got.c_hsc},
                {"c_reb", got.c_reb},        {"c_hbo", got.c_hbo},
                {"frac_lsc", got.frac_lsc},  {"frac_hsc", got.frac_hsc},
                {"frac_reb", got.frac_reb},  {"frac_hbo", got.frac_hbo},
                {"lockout_firm", 0.0}};
        const Value::Array &rows = array_of(
                member(doc, "shock_coefficients", "doc"), "shock coeffs");
        bt::require(rows.size() == flat.size(), "shock coeff count");
        for (std::size_t i = 0; i < flat.size(); ++i) {
            const Value::Array &kv = array_of(rows.at(i), "kv");
            bt::require(string_of(kv.at(0), "kv") == flat.at(i).first,
                        "shock coeff order " + std::to_string(i));
            if (flat.at(i).first == "lockout_firm")
                bt::require(bool_of(kv.at(1), "kv") ==
                                    got.lockout_firm,
                            "lockout_firm bool");
            else
                bt::require(same_bits(flat.at(i).second,
                                      scalar_field(kv.at(1), "kv")),
                            "shock coeff bits " + flat.at(i).first);
        }
    }

    {
        const Value &fc = member(doc, "fork_ctor_record", "doc");
        const mdl::Charger3Damper d;
        bt::require(d.hsc_clicks == 2 && d.lsc_clicks == 7 &&
                            d.rebound_clicks == 9,
                    "fork ctor clicks");
        expect_scalar_bits(member(fc, "hbo_start_mm", "fc"),
                           d.hbo_start_mm, "fork hbo_start");
        expect_scalar_bits(member(fc, "c_hbo_base", "fc"), d.c_hbo_base,
                           "fork c_hbo_base");
        bt::require(bool_of(member(fc, "legacy_behavior", "fc"), "fc") ==
                            d.legacy_behavior,
                    "fork legacy flag");
        const Value &sc = member(doc, "shock_ctor_record", "doc");
        const mdl::SuperDeluxeDamper s;
        bt::require(s.hsc_clicks == 2 && s.lsc_clicks == 7 &&
                            s.rebound_clicks == 7 && s.hbo_clicks == 2 &&
                            !s.lockout_firm,
                    "shock ctor clicks");
        expect_scalar_bits(member(sc, "hbo_start_mm", "sc"),
                           s.hbo_start_mm, "shock hbo_start");
        bt::require(static_cast<int>(
                            number_of(member(sc, "max_hbo", "sc"))) ==
                            s.max_hbo,
                    "shock max_hbo");
        expect_scalar_bits(member(sc, "total_stroke_mm", "sc"),
                           s.total_stroke_mm, "shock stroke");
        expect_scalar_bits(member(sc, "c_hbo_min", "sc"), s.c_hbo_min,
                           "shock c_hbo_min");
        expect_scalar_bits(member(sc, "c_hbo_max", "sc"), s.c_hbo_max,
                           "shock c_hbo_max");
        expect_scalar_bits(member(sc, "lockout_preload_n", "sc"),
                           s.lockout_preload_n, "shock preload");
        expect_scalar_bits(member(sc, "lockout_stiffness", "sc"),
                           s.lockout_stiffness, "shock lockout k");
        bt::require(bool_of(member(sc, "legacy_behavior", "sc"), "sc") ==
                            s.legacy_behavior,
                    "shock legacy flag");
    }

    const mdl::Charger3Damper fork;
    const Value::Array &frows =
            array_of(member(doc, "fork_force_rows", "doc"), "doc");
    for (const Value &c: frows) {
        const double v =
                scalar_field(member(c, "velocity_mps", "c"), "c");
        const double t =
                scalar_field(member(c, "travel_mm", "c"), "c");
        bt::require(outcome_accepted(c, "fork row"), "fork row accepts");
        expect_scalar_bits(*outcome_result(c, "fork row"),
                           fork.compute_damping_force(v, t), "fork row");
    }

    const mdl::SuperDeluxeDamper shock;
    const Value::Array &srows =
            array_of(member(doc, "shock_force_rows", "doc"), "doc");
    for (const Value &c: srows) {
        const double v =
                scalar_field(member(c, "velocity_mps", "c"), "c");
        const double s =
                scalar_field(member(c, "stroke_mm", "c"), "c");
        const Value &comp = member(c, "components", "shock row");
        bt::require(outcome_accepted(comp, "comp"), "comp accepts");
        const mdl::SuperDeluxeDamper::Components got =
                shock.compute_damping_components(v, s);
        const PairExpectation want = pairs_of(
                *outcome_result(comp, "comp"), "comp");
        bt::require(want.names.size() == 2 &&
                            want.names.at(0) == "base_n" &&
                            want.names.at(1) == "hbo_n",
                    "component key order");
        bt::require(same_bits(got.base_n, want.values.at(0)) &&
                            same_bits(got.hbo_n, want.values.at(1)),
                    "component bits");
        const Value &force = member(c, "force", "shock row");
        bt::require(outcome_accepted(force, "force"),
                        "force accepts");
        expect_scalar_bits(*outcome_result(force, "force"),
                           shock.compute_damping_force(v, s),
                           "shock force");
    }

}

void test_damper_mutations() {
    const Value doc = doc_of("model/dampers");
    {
        const mdl::SuperDeluxeDamper locked([] {
            mdl::SuperDeluxeDamper d;
            d.lockout_firm = true;
            return d;
        }());
        const mdl::SuperDeluxeDamper legacy_shock(
                2, 7, 7, 2, false, 65.0, true);
        const Value::Array &lrows =
                array_of(member(doc, "lockout_rows", "doc"), "doc");
        for (const Value &c: lrows) {
            const double v =
                    scalar_field(member(c, "velocity_mps", "c"), "c");
            const Value &modern = member(c, "modern", "lockout");
            bt::require(outcome_accepted(modern, "modern"),
                        "modern accepts");
            expect_scalar_bits(*outcome_result(modern, "modern"),
                               locked.compute_damping_force(v, 20.0),
                               "lockout modern");
            const Value &legacy = member(c, "legacy", "lockout");
            bt::require(outcome_accepted(legacy, "legacy"),
                        "legacy accepts");
            expect_scalar_bits(*outcome_result(legacy, "legacy"),
                               legacy_shock.compute_damping_force(v,
                                                                  20.0),
                               "lockout legacy");
        }
    }

    const Value::Array &clicks =
            array_of(member(doc, "click_cases", "doc"), "doc");
    for (const Value &c: clicks) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const std::string label = "click " + tag;
        if (tag == "fork_set_within") {
            mdl::Charger3Damper d;
            d.set_clicks(3, 5, std::nullopt);
            bt::require(d.hsc_clicks == 3 && d.lsc_clicks == 5 &&
                                d.rebound_clicks == 9,
                        label);
        } else if (tag == "fork_clamps") {
            mdl::Charger3Damper d;
            d.set_clicks(99, -3, 25);
            bt::require(d.hsc_clicks == 4 && d.lsc_clicks == 0 &&
                                d.rebound_clicks == 17,
                        label);
        } else if (tag == "fork_bad_click_type") {
            // set_clicks(hsc=1.5) — the non-integer input is a
            // Python-domain branch; optional<int> cannot carry it.
            bt::require(!outcome_accepted(c, label),
                        std::string(label) + " stays rejected");
        } else if (tag == "shock_set_all") {
            mdl::SuperDeluxeDamper d;
            d.set_clicks(1, 2, 3, 4, true);
            bt::require(d.hsc_clicks == 1 && d.lsc_clicks == 2 &&
                                d.rebound_clicks == 3 &&
                                d.hbo_clicks == 4 && d.lockout_firm,
                        label);
        } else if (tag == "toggle_lockout") {
            mdl::SuperDeluxeDamper d;
            const bool first = d.toggle_lockout();
            const bool second = d.toggle_lockout();
            bt::require(first && !second && !d.lockout_firm, label);
        } else if (tag == "fork_ctor_clamps") {
            const mdl::Charger3Damper d(99, -1, 99);
            bt::require(d.hsc_clicks == 4 && d.lsc_clicks == 0 &&
                                d.rebound_clicks == 17,
                        label);
        } else if (tag == "fork_low_travel") {
            expect_invalid_msg(
                    outcome_message(c, label),
                    [] { const mdl::Charger3Damper d(2, 7, 9, 10.0); },
                    label);
        } else if (tag == "fork_low_travel_legacy") {
            const mdl::Charger3Damper d(2, 7, 9, 10.0, true);
            bt::require(d.hsc_clicks == 2 && d.lsc_clicks == 7 &&
                                d.rebound_clicks == 9,
                        label);
        } else if (tag == "shock_zero_stroke") {
            expect_invalid_msg(
                    outcome_message(c, label),
                    [] { const mdl::SuperDeluxeDamper d(2, 7, 7, 2, false, 0.0); },
                    label);
        } else if (tag == "fork_nan_velocity") {
            expect_invalid_msg(
                    outcome_message(c, label),
                    [] {
                        (void)mdl::Charger3Damper{}.compute_damping_force(
                                std::numeric_limits<double>::quiet_NaN(),
                                50.0);
                    },
                    label);
        } else if (tag == "shock_nan_stroke") {
            expect_invalid_msg(
                    outcome_message(c, label),
                    [] {
                        (void)mdl::SuperDeluxeDamper{}
                                .compute_damping_force(
                                        0.5,
                                        std::numeric_limits<double>::
                                                quiet_NaN());
                    },
                    label);
        } else {
            bt::require(false, "click case tag: " + tag);
        }
    }

}

void test_damper_presets_apply() {
    const Value doc = doc_of("model/dampers");
    {
        const Value &df = member(doc, "dyno_fork", "doc");
        const mdl::Charger3Damper::DynoCurve dyno =
                mdl::Charger3Damper{}.compute_dyno_curve(1.5, 41);
        expect_bits(member(df, "velocity_mps", "dyno"), dyno.velocity_mps,
                    "dyno fork vel");
        expect_bits(member(df, "force_n", "dyno"), dyno.force_n,
                    "dyno fork force");
        bt::require(dyno.hsc_clicks == 2 && dyno.lsc_clicks == 7 &&
                            dyno.rebound_clicks == 9,
                    "dyno fork clicks");
        const Value &ds = member(doc, "dyno_shock", "doc");
        const mdl::SuperDeluxeDamper::DynoCurve dyno_s =
                mdl::SuperDeluxeDamper{}.compute_dyno_curve(1.5, 41);
        expect_bits(member(ds, "velocity_mps", "dyno"),
                    dyno_s.velocity_mps, "dyno shock vel");
        expect_bits(member(ds, "force_n", "dyno"), dyno_s.force_n,
                    "dyno shock force");
        bt::require(dyno_s.hsc_clicks == 2 && dyno_s.lsc_clicks == 7 &&
                            dyno_s.rebound_clicks == 7 &&
                            dyno_s.hbo_clicks == 2 && !dyno_s.lockout,
                    "dyno shock clicks");
    }

    const Value::Array &applied =
            array_of(member(doc, "apply_presets", "doc"), "doc");
    for (const Value &c: applied) {
        const std::string &name =
                string_of(member(c, "preset", "c"), "c");
        const mdl::DamperPreset p = preset_named(name);
        bt::require(outcome_accepted(c, "apply " + name),
                    "apply accepts: " + name);
        mdl::BikeSuspensionSystem s;
        s.apply_preset(p);
        const Value &res = *outcome_result(c, "apply " + name);
        bt::require(string_of(member(res, "preset", "res"), "res") ==
                            preset_name(s.current_preset),
                    "applied preset");
        const Value::Array &fc =
                array_of(member(res, "fork_clicks", "res"), "res");
        bt::require(fc.size() == 3 &&
                            static_cast<int>(number_of(fc.at(0), "fc")) ==
                                    s.fork_damper.hsc_clicks &&
                            static_cast<int>(number_of(fc.at(1), "fc")) ==
                                    s.fork_damper.lsc_clicks &&
                            static_cast<int>(number_of(fc.at(2), "fc")) ==
                                    s.fork_damper.rebound_clicks,
                    "applied fork clicks");
        const Value::Array &sc =
                array_of(member(res, "shock_clicks", "res"), "res");
        bt::require(sc.size() == 4 &&
                            static_cast<int>(number_of(sc.at(0), "sc")) ==
                                    s.shock_damper.hsc_clicks &&
                            static_cast<int>(number_of(sc.at(1), "sc")) ==
                                    s.shock_damper.lsc_clicks &&
                            static_cast<int>(number_of(sc.at(2), "sc")) ==
                                    s.shock_damper.rebound_clicks &&
                            static_cast<int>(number_of(sc.at(3), "sc")) ==
                                    s.shock_damper.hbo_clicks,
                    "applied shock clicks");
        bt::require(bool_of(member(res, "lockout", "res"), "res") ==
                            s.shock_damper.lockout_firm,
                    "applied lockout");
        expect_click_config(member(res, "config", "res"), s.config,
                            "applied config");
    }

    {
        const Value::Array &cycle =
                array_of(member(doc, "cycle_sequence", "doc"), "doc");
        mdl::BikeSuspensionSystem s;
        for (const Value &step: cycle) {
            const bool forward =
                    bool_of(member(step, "forward", "step"), "step");
            const std::string &want =
                    string_of(member(step, "preset", "step"), "step");
            if (forward == true && want == "PLUSH" &&
                &step == &cycle.back()) {
                // The final row follows current_preset = CUSTOM — the
                // oracle's preset_list.index raises ValueError and the
                // cycle restarts from index 0.
                s.current_preset = mdl::DamperPreset::custom;
            }
            bt::require(preset_name(s.cycle_preset(forward)) == want,
                        "cycle step -> " + want);
        }
    }
}

// ---- model/tuning ----------------------------------------------------------------

void test_tuning() {
    const Value doc = doc_of("model/tuning");

    const Value::Array &refl =
            array_of(member(doc, "reflected_cases", "doc"), "doc");
    for (const Value &c: refl) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        const double m =
                scalar_field(member(c, "mass_kg", "c"), "c");
        const double lr =
                scalar_field(member(c, "leverage_ratio", "c"), "c");
        if (outcome_accepted(c, "refl " + tag)) {
            expect_scalar_bits(*outcome_result(c, "refl " + tag),
                               mdl::reflected_shock_mass(m, lr),
                               "refl " + tag);
        } else {
            expect_invalid_msg(
                    outcome_message(c, "refl " + tag),
                    [&] { (void)mdl::reflected_shock_mass(m, lr); },
                    "refl " + tag);
        }
    }

    const geo::BikeSpecs specs;
    const mdl::BikeMassSpecs mass_specs;
    const mdl::RiderSpecs rider_default{};
    const Value::Array &cases =
            array_of(member(doc, "tuning_cases", "doc"), "doc");
    for (const Value &c: cases) {
        const std::string &tag = string_of(member(c, "tag", "c"), "c");
        mdl::SuspensionTuning got{};
        if (tag == "default")
            got = mdl::compute_suspension_tuning_for_sag(specs,
                                                       mass_specs);
        else if (tag == "rider_90kg")
            got = mdl::compute_suspension_tuning_for_sag(
                    specs, mass_specs, nullptr, 90.0);
        else if (tag == "seated_rider")
            got = mdl::compute_suspension_tuning_for_sag(
                    specs, mass_specs, &rider_default);
        else if (tag == "front_fraction")
            got = mdl::compute_suspension_tuning_for_sag(
                    specs, mass_specs, nullptr, std::nullopt, 30.0,
                    30.0, 0.45);
        else if (tag == "custom_sag")
            got = mdl::compute_suspension_tuning_for_sag(
                    specs, mass_specs, nullptr, std::nullopt, 25.0,
                    33.0);
        else
            bt::require(false, "tuning tag: " + tag);
        bt::require(outcome_accepted(c, "tuning " + tag),
                    "tuning case must accept: " + tag);
        const Value::Array &pairs = array_of(
                *outcome_result(c, "tuning " + tag), "tuning " + tag);
        static const std::array<std::string_view, 19> kKeys{
                "total_system_mass_kg",       "front_load_fraction",
                "target_front_sag_mm",        "fork_stiffness_n_m",
                "fork_damping_n_s_m",         "fork_air_tokens",
                "fork_calibrated_psi",        "fork_air_force_full_travel_n",
                "fork_air_bottom_out_g",      "fork_air_progressivity_pct",
                "front_spring_force_full_travel_n", "front_bottom_out_g",
                "target_rear_sag_mm",         "shock_stroke_at_sag_mm",
                "lr_at_sag",                  "shock_stiffness_n_m",
                "shock_damping_n_s_m",
                "rear_spring_force_full_travel_n", "rear_bottom_out_g"};
        const std::vector<double> flat{
                got.total_system_mass_kg,       got.front_load_fraction,
                got.target_front_sag_mm,        got.fork_stiffness_n_m,
                got.fork_damping_n_s_m,
                static_cast<double>(got.fork_air_tokens),
                got.fork_calibrated_psi,
                got.fork_air_force_full_travel_n, got.fork_air_bottom_out_g,
                got.fork_air_progressivity_pct,
                got.front_spring_force_full_travel_n,
                got.front_bottom_out_g,         got.target_rear_sag_mm,
                got.shock_stroke_at_sag_mm,     got.lr_at_sag,
                got.shock_stiffness_n_m,        got.shock_damping_n_s_m,
                got.rear_spring_force_full_travel_n,
                got.rear_bottom_out_g};
        bt::require(pairs.size() == kKeys.size(), "tuning key count");
        for (std::size_t i = 0; i < kKeys.size(); ++i) {
            const Value::Array &kv = array_of(pairs.at(i), "kv");
            bt::require(string_of(kv.at(0), "kv") == kKeys.at(i),
                        "tuning key order at " + std::to_string(i));
            if (kKeys.at(i) == "fork_air_tokens") {
                bt::require(static_cast<int>(number_of(kv.at(1), "kv")) ==
                                    got.fork_air_tokens,
                            "fork_air_tokens");
            } else {
                bt::require(same_bits(flat.at(i),
                                      scalar_field(kv.at(1), "kv")),
                            "tuning bits " + std::string(kKeys.at(i)));
            }
        }
    }
}

// ---- model/mass_properties ---------------------------------------------------------

void test_mass_properties() {
    // The mjb is one of the manifest-verified files for this fixture;
    // doc_of already validated every declared byte + SHA-256.
    const Value doc = doc_of("model/mass_properties");

    using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
    using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
    const ModelPtr model(
            engine::load_model(
                    bt::fixture("model/wheel_contacts.mjb")
                            .string()
                            .c_str()),
            &mj_deleteModel);
    bt::require(model != nullptr, "wheel_contacts.mjb must load");
    const DataPtr data(engine::make_data(model.get()), &mj_deleteData);
    engine::forward(model.get(), data.get());

    bt::require(static_cast<int>(number_of(
                        member(doc, "ncon", "doc"))) == data->ncon,
                "ncon");
    bt::require(static_cast<int>(number_of(
                        member(doc, "nbody", "doc"))) == model->nbody,
                "nbody");
    const int front_gid = mj_name2id(model.get(), mjOBJ_GEOM,
                                     "front_wheel");
    const int rear_gid = mj_name2id(model.get(), mjOBJ_GEOM, "rear_wheel");
    bt::require(front_gid == static_cast<int>(number_of(
                                     member(doc, "front_geom_id", "doc"))),
                "front geom id");
    bt::require(rear_gid == static_cast<int>(number_of(
                                    member(doc, "rear_geom_id", "doc"))),
                "rear geom id");

    // Sanity: the compiled model/image matches the recorded lanes
    // bitwise before comparing the ported reads.
    {
        std::vector<double> body_mass(
                static_cast<std::size_t>(model->nbody));
        std::copy_n(model->body_mass, model->nbody, body_mass.begin());
        expect_bits(member(doc, "body_mass", "doc"), body_mass,
                    "body_mass");
        std::vector<double> xipos(
                static_cast<std::size_t>(model->nbody) * 3);
        std::copy_n(data->xipos,
                    static_cast<std::size_t>(model->nbody) * 3,
                    xipos.begin());
        expect_bits(member(doc, "xipos", "doc"), xipos, "xipos");
    }

    expect_bits(member(doc, "compiled_com", "doc"),
                mdl::compiled_center_of_mass(*model, *data),
                "compiled_com");
    const auto [front_load, rear_load] =
            mdl::static_contact_loads(*model, *data, front_gid, rear_gid);
    const std::vector<double> loads{front_load, rear_load};
    const std::vector<double> want_loads =
            scalar_list(member(doc, "contact_loads", "doc"), "loads");
    bt::require(loads.size() == want_loads.size(), "load arity");
    for (std::size_t i = 0; i < loads.size(); ++i)
        bt::require(same_bits(loads.at(i), want_loads.at(i)),
                    "contact load " + std::to_string(i));

    const Value &no_contact = member(doc, "no_contact_error", "doc");
    {
        const DataPtr fresh(engine::make_data(model.get()),
                            &mj_deleteData);
        expect_invalid_msg(
                string_of(member(no_contact, "message", "doc"), "doc"),
                [&] {
                    (void)mdl::static_contact_loads(*model, *fresh,
                                                    front_gid, rear_gid);
                },
                "no_contact_error");
    }
    const Value &swapped = member(doc, "swapped_ids_error", "doc");
    expect_invalid_msg(
            string_of(member(swapped, "message", "doc"), "doc"),
            [&] {
                (void)mdl::static_contact_loads(*model, *data, rear_gid,
                                                front_gid);
            },
            "swapped_ids_error");
}

} // namespace

int main() {
    return bt::run([] {
        test_inertia();
        test_component_masses();
        test_mass_specs();
        test_component_centers();
        test_static_system_cg();
        test_rider_specs();
        test_seated_pose();
        test_rider_helpers();
        test_rider_legs_centers();
        test_rider_segments();
        test_air_spring();
        test_coil_shock();
        test_damper_presets();
        test_damper_mutations();
        test_damper_presets_apply();
        test_tuning();
        test_mass_properties();
    });
}
