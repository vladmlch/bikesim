// test_native_rider_contact_math.cpp — C++ port of
// tests/reference/test_native_rider_contact_math.py (batch 3.14a, T52).
//
// Replays rider::box_pad_contact / upper_box_face / sole_target_height /
// project_sole_goal / grip_step / release_if_overloaded / hypot3 /
// validate_planar_support_model against the frozen two-platform goldens.
// Every numpy-synthesized input (RNG draws, matmul-formed centers) arrives
// through 'input.*' replay entries where the oracle's lowering could drift
// across BLAS builds; literal construction stays C++-side.
#include <cmath>

#include "diag.hpp"
#include "rider_contacts_reference_support.hpp"

namespace {

using namespace rider_contacts_reference;
constexpr std::string_view kFile = "test_native_rider_contact_math.py";

// rotation(angle) — the oracle's Ry matrix, row-major. Mat3 is a 72
// byte value object by design; the by-value return is the literal
// helper's one permitted large copy.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
[[nodiscard]] rider::Mat3 rotation(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return {c, 0., s, 0., 1., 0., -s, 0., c};
}
NATIVE_DIAG_POP

// The six box_pad_oracle dict leaves in binding insertion order.
void box_pad_leaves(bt::oracle::Node &golden,
                    const rider::BoxPadContact &actual) {
    vec3(golden, actual.point_m, "");
    vec3(golden, actual.normal, "");
    vec3(golden, actual.tangent(), "");
    scalar(golden, actual.gap_m, "");
    boolean(golden, actual.within_width, "within_width");
    boolean(golden, actual.within_footprint, "within_footprint");
}

// ---------------------------------------------------------------------
// test_box_pad_oracle[local{i}-{angle}] — 8 locals x 6 angles.
// ---------------------------------------------------------------------
void test_box_pad_oracle(std::size_t local_index, const rider::Vec3 &local,
                         double angle, std::string_view angle_repr) {
    auto golden = node(kFile, "test_box_pad_oracle",
                       "local" + std::to_string(local_index) + "-" +
                               std::string(angle_repr));
    const rider::Vec3 origin{.3, -.2, .5};
    const rider::Vec3 half{.1, .04, .02};
    const rider::Mat3 rot = rotation(angle);
    // center = origin + rot @ local — the oracle's matmul lowering.
    const rider::Vec3 center =
            rider::add(origin, rider::matvec(rot, local));
    const auto actual =
            rider::box_pad_contact(center, .025, origin, rot, half);
    box_pad_leaves(golden, actual);
    golden.done();
}

// ---------------------------------------------------------------------
// test_box_random_and_medial_ties — 200 'input.*' replayed cases plus the
// three medial-tie normals on axis-aligned origins.
// ---------------------------------------------------------------------
void test_box_random_and_medial_ties() {
    auto golden = node(kFile, "test_box_random_and_medial_ties");
    for (int i = 0; i < 200; ++i) {
        const auto rot_flat = input_vector(golden, "input.rot");
        const auto origin_v = input_vector(golden, "input.origin");
        const auto half_v = input_vector(golden, "input.half");
        const auto center_v = input_vector(golden, "input.center");
        rider::Mat3 rot;
        std::copy_n(rot_flat.begin(), 9, rot.begin());
        rider::Vec3 origin, half, center;
        std::copy_n(origin_v.begin(), 3, origin.begin());
        std::copy_n(half_v.begin(), 3, half.begin());
        std::copy_n(center_v.begin(), 3, center.begin());
        const auto actual =
                rider::box_pad_contact(center, .02, origin, rot, half);
        vec3(golden, actual.point_m, "");
        vec3(golden, actual.normal, "");
        vec3(golden, actual.tangent(), "");
        scalar(golden, actual.gap_m, "");
        boolean(golden, actual.within_width, "within_width");
        boolean(golden, actual.within_footprint, "within_footprint");
    }
    for (const double angle : {0., M_PI / 2., M_PI}) {
        const auto actual =
                rider::box_pad_contact({0., 0., 0.}, .01, {0., 0., 0.},
                                       rotation(angle), {.1, .1, .1});
        vec3(golden, actual.normal, "");
    }
    golden.done();
}

// ---------------------------------------------------------------------
// test_upper_face_oracle[angle] — the (point, normal, tangent) triple.
// ---------------------------------------------------------------------
void test_upper_face_oracle(double angle, std::string_view angle_repr) {
    auto golden = node(kFile, "test_upper_face_oracle",
                       std::string(angle_repr));
    const auto face = rider::upper_box_face({.3, .2, -.4}, rotation(angle),
                                            {.1, .04, .02});
    vec3(golden, face.point_m, "");
    vec3(golden, face.normal, "");
    vec3(golden, face.tangent, "");
    golden.done();
}

// ---------------------------------------------------------------------
// test_sole_height_oracle[sole_x-compression-angle] — scalar height or the
// recorded exception class+message.
// ---------------------------------------------------------------------
void test_sole_height_oracle(double sole_x, std::string_view sole_x_repr,
                             double compression,
                             std::string_view compression_repr,
                             double angle, std::string_view angle_repr) {
    auto golden = node(kFile, "test_sole_height_oracle",
                       std::string(sole_x_repr) + "-" +
                               std::string(compression_repr) + "-" +
                               std::string(angle_repr));
    const rider::Vec3 origin{0., 0., 0.};
    const rider::Vec3 half{.1, .04, .02};
    const rider::Mat3 rot = rotation(angle);
    try {
        const double actual = rider::sole_target_height(
                origin, rot, half, sole_x, .035, .012, compression);
        scalar(golden, actual, "");
    } catch (const rider::UnreachableSoleTarget &failure) {
        boolean(golden, true, "outcome.unreachable");
        text_leaf(golden, failure.what(), "outcome.message");
    } catch (const std::runtime_error &failure) {
        boolean(golden, false, "outcome.unreachable");
        text_leaf(golden, failure.what(), "outcome.message");
    }
    golden.done();
}

// ---------------------------------------------------------------------
// test_project_goal_oracle[compression-shear-angle] — goal + diagnostic
// dict, or the recorded exception.
// ---------------------------------------------------------------------
void test_project_goal_oracle(double compression, double shear,
                              std::string_view pair_repr, double angle,
                              std::string_view angle_repr) {
    auto golden = node(kFile, "test_project_goal_oracle",
                       std::string(pair_repr) + "-" +
                               std::string(angle_repr));
    const rider::Vec3 origin{.3, -.2, .5};
    const rider::Vec3 half{.1, .04, .02};
    const rider::Mat3 rot = rotation(angle);
    try {
        const auto actual = rider::project_sole_goal(
                origin, rot, half, .035, .012, compression, shear);
        vec3(golden, actual.goal, "");
        const auto &d = actual.diagnostic;
        scalar(golden, d.requested_compression_m, "");
        scalar(golden, d.applied_compression_m, "");
        scalar(golden, d.requested_shear_m, "");
        scalar(golden, d.applied_shear_m, "");
        boolean(golden, d.saturated, "saturated");
        for (std::size_t i = 0; i < d.limiting_reasons.size(); ++i)
            text_leaf(golden, d.limiting_reasons[i],
                      "limiting_reasons[" + std::to_string(i) + "]");
    } catch (const rider::UnreachableSoleTarget &failure) {
        boolean(golden, true, "outcome.unreachable");
        text_leaf(golden, failure.what(), "outcome.message");
    } catch (const std::runtime_error &failure) {
        boolean(golden, false, "outcome.unreachable");
        text_leaf(golden, failure.what(), "outcome.message");
    }
    golden.done();
}

// ---------------------------------------------------------------------
// test_grip_energy_oracle[k-c-dt] — 2 literals + 150 'input.pair' pairs,
// each emitting (xi, force, energy_j, loss_j).
// ---------------------------------------------------------------------
void test_grip_energy_oracle(double k, double c, double dt,
                             std::string_view params) {
    auto golden = node(kFile, "test_grip_energy_oracle",
                       std::string(params));
    const std::array literals{
            std::pair{rider::Vec3{.003, -.002, .001},
                      rider::Vec3{.1, 0., -.2}},
            std::pair{rider::Vec3{-0., 0., -0.}, rider::Vec3{0., 0., 0.}}};
    const int total = 2 + 150;
    for (int i = 0; i < total; ++i) {
        rider::Vec3 xi{}, velocity{};
        if (i < 2) {
            xi = literals[static_cast<std::size_t>(i)].first;
            velocity = literals[static_cast<std::size_t>(i)].second;
        } else {
            const auto pair = input_vector(golden, "input.pair");
            std::copy_n(pair.begin(), 3, xi.begin());
            std::copy_n(pair.begin() + 3, 3, velocity.begin());
        }
        const auto actual = rider::grip_step(xi, velocity, k, c, dt);
        vec3(golden, actual.xi, "");
        vec3(golden, actual.force, "");
        scalar(golden, actual.energy_j, "");
        scalar(golden, actual.loss_j, "");
    }
    golden.done();
}

// ---------------------------------------------------------------------
// test_grip_release_oracle[limit-force{i}] — (force, loss_j, overloaded).
// ---------------------------------------------------------------------
void test_grip_release_oracle(const rider::Vec3 &force,
                              std::size_t force_index, double limit,
                              std::string_view limit_repr) {
    auto golden = node(kFile, "test_grip_release_oracle",
                       std::string(limit_repr) + "-force" +
                               std::to_string(force_index));
    const auto actual = rider::release_if_overloaded(force, .2, limit);
    vec3(golden, actual.force, "");
    scalar(golden, actual.loss_j, "");
    boolean(golden, actual.overloaded, "overloaded");
    golden.done();
}

// ---------------------------------------------------------------------
// test_three_coordinate_hypot_oracle — 5 literals + 1000 'input.value'
// triples; NaN results record the |b1 'value' flag.
// ---------------------------------------------------------------------
void test_three_coordinate_hypot_oracle() {
    auto golden = node(kFile, "test_three_coordinate_hypot_oracle");
    const std::array literals{rider::Vec3{0., -0., 0.},
                              rider::Vec3{1e308, 1e308, 1e308},
                              rider::Vec3{5e-324, 5e-324, 5e-324},
                              rider::Vec3{
                                      std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      1.},
                              rider::Vec3{
                                      std::numeric_limits<double>::quiet_NaN(),
                                      1., 2.}};
    for (std::size_t i = 0; i < literals.size() + 1000; ++i) {
        rider::Vec3 value{};
        if (i < literals.size()) {
            value = literals[i];
        } else {
            const auto v = input_vector(golden, "input.value");
            std::copy_n(v.begin(), 3, value.begin());
        }
        const double actual = rider::hypot3(value);
        if (std::isnan(actual)) {
            boolean(golden, true, "value");
        } else {
            scalar(golden, actual, "");
        }
    }
    golden.done();
}

// ---------------------------------------------------------------------
// test_invalid_grip[...] — the C++-expressible domain rejections; the
// marshal-only rows (width-2 xi) are nanobind shape contracts documented
// in the port-map record.
// ---------------------------------------------------------------------
void test_invalid_grip() {
    error<std::invalid_argument>(
            [] {
                rider::grip_step({0., std::numeric_limits<double>::quiet_NaN(),
                                  0.},
                                 {0., 0., 0.}, 1., 0., .01);
            },
            "", std::source_location::current());
    error<std::invalid_argument>(
            [] { rider::grip_step({0., 0., 0.}, {0., 0., 0.}, 0., 0., .01); });
    error<std::invalid_argument>(
            [] { rider::grip_step({0., 0., 0.}, {0., 0., 0.}, 1., -1., .01); });
    error<std::invalid_argument>(
            [] { rider::grip_step({0., 0., 0.}, {0., 0., 0.}, 1., 0., 0.); });
    error<std::invalid_argument>([] {
        rider::release_if_overloaded(
                {0., std::numeric_limits<double>::infinity(), 0.}, 0., 1.);
    });
    error<std::invalid_argument>(
            [] { rider::release_if_overloaded({0., 0., 0.}, -1., 1.); });
    error<std::invalid_argument>(
            [] { rider::release_if_overloaded({0., 0., 0.}, 0., 0.); });
}

// test_grip_overflow — 'grip update overflow' on the 1e308 inputs.
void test_grip_overflow() {
    error<std::invalid_argument>(
            [] {
                rider::grip_step({1e308, 1e308, 1e308}, {1e308, 1e308, 1e308},
                                 1e308, 0., .1);
            },
            "grip update overflow");
}

// test_invalid_box[...] — the valid-domain rejections the C++ signature
// can express; marshal-only rows (width-2 center, 2x2 rotation) are
// binding shape contracts documented in the port-map record.
void test_invalid_box() {
    error<std::invalid_argument>([] {
        rider::box_pad_contact({0., std::numeric_limits<double>::quiet_NaN(),
                                0.},
                               .01, {0., 0., 0.},
                               rider::Mat3{1., 0., 0., 0., 1., 0., 0., 0., 1.},
                               {1., 1., 1.});
    });
    error<std::invalid_argument>([] {
        rider::box_pad_contact({0., 0., 0.}, 0., {0., 0., 0.},
                               rider::Mat3{1., 0., 0., 0., 1., 0., 0., 0., 1.},
                               {1., 1., 1.});
    });
    error<std::invalid_argument>([] {
        rider::box_pad_contact({0., 0., 0.}, .01,
                               {std::numeric_limits<double>::infinity(), 0.,
                                0.},
                               rider::Mat3{1., 0., 0., 0., 1., 0., 0., 0., 1.},
                               {1., 1., 1.});
    });
    error<std::invalid_argument>([] {
        rider::box_pad_contact({0., 0., 0.}, .01, {0., 0., 0.},
                               rider::Mat3{1., 0., 0., 0., 1., 0., 0., 0., 1.},
                               {0., .1, .1});
    });
    error<std::invalid_argument>([] {
        rider::box_pad_contact(
                {0., 0., 0.}, .01, {0., 0., 0.},
                rider::Mat3{-1., 0., 0., 0., 1., 0., 0., 0., 1.}, {1., 1., 1.});
    });
    error<std::invalid_argument>([] {
        rider::box_pad_contact(
                {0., 0., 0.}, .01, {0., 0., 0.},
                rider::Mat3{1., 0., 0., 0., 0., -1., 0., 1., 0.}, {1., 1., 1.});
    });
}

// ---------------------------------------------------------------------
// test_outputs_own_storage_and_convert_dtype — value semantics: the
// mutation checks are compile-time ownership in C++ (struct returns),
// the recorded leaves pin the arithmetic.
// ---------------------------------------------------------------------
void test_outputs_own_storage_and_convert_dtype() {
    auto golden = node(kFile, "test_outputs_own_storage_and_convert_dtype");
    // float32 inputs: np.asarray upcasts — identical f64 payloads.
    const auto step = rider::grip_step({1., 2., 3.}, {0., 2., 4.}, 1., 0., .01);
    vec3(golden, step.xi, "");
    vec3(golden, step.force, "");
    scalar(golden, step.energy_j, "");
    scalar(golden, step.loss_j, "");
    const auto first = rider::box_pad_contact(
            {0., 0., 0.}, .01, {0., 0., 0.},
            rider::Mat3{1., 0., 0., 0., 1., 0., 0., 0., 1.}, {1., 1., 1.});
    auto mutated = first;
    mutated.normal = {999., 999., 999.};
    const auto second = rider::box_pad_contact(
            {0., 0., 0.}, .01, {0., 0., 0.},
            rider::Mat3{1., 0., 0., 0., 1., 0., 0., 0., 1.}, {1., 1., 1.});
    vec3(golden, second.normal, "");
    bt::require(bitwise_equal(first.point_m, second.point_m) &&
                        bitwise_equal(first.normal, second.normal),
                "box_pad_contact is deterministic across calls");
    golden.done();
}

// ---------------------------------------------------------------------
// test_planar_model_validation_oracle[joint-geom-quat] — the tiny
// two-body model; recorded 'outcome.message' on the rejecting rows.
// ---------------------------------------------------------------------
struct PlanarCase {
    std::string_view joint;
    std::string_view geom;
    std::string_view quat;
    bool rejects;
};

void test_planar_model_validation_oracle(const bt::TemporaryDirectory &dir,
                                         const PlanarCase &test_case) {
    const std::string xml =
            "<mujoco><worldbody><body>" + std::string(test_case.joint) +
            "<geom name=\"support\" type=\"" + std::string(test_case.geom) +
            "\" size=\".1 .04 .02\" quat=\"" + std::string(test_case.quat) +
            "\" mass=\"1\"/></body><body pos=\"1 0 1\"><freejoint/>"
            "<geom type=\"sphere\" size=\".1\" mass=\"1\"/></body>"
            "</worldbody></mujoco>";
    auto model = bike::model::compile_mjcf(xml);
    const auto mjb =
            (dir.path() / "support.mjb").string();
    mj_saveModel(model.get(), mjb.c_str(), nullptr, 0);
    Stepper stepper(mjb);
    stepper.forward();
    const int geom_id = mj_name2id(model.get(), mjOBJ_GEOM, "support");
    model_access::require_id(geom_id, model->ngeom, "support");
    const std::array ids{geom_id};
    const std::string params = std::string(test_case.joint) + "-" +
                               std::string(test_case.geom) + "-" +
                               std::string(test_case.quat);
    try {
        rider::validate_planar_support_model(stepper.model(),
                                             stepper.data(), ids);
        bt::require(!test_case.rejects,
                    "planar support validation accepted a recorded "
                    "rejection case: " + params);
    } catch (const std::invalid_argument &failure) {
        bt::require(test_case.rejects,
                    "planar support validation rejected an accepted case: " +
                            params);
        auto golden = node(kFile, "test_planar_model_validation_oracle",
                           params);
        text_leaf(golden, failure.what(), "outcome.message");
        golden.done();
    }
    const std::array bad_low{-1};
    const std::array bad_high{static_cast<int>(model->ngeom)};
    error<std::invalid_argument>(
            [&] {
                rider::validate_planar_support_model(stepper.model(),
                                                     stepper.data(), bad_low);
            },
            "invalid rider support geom");
    error<std::invalid_argument>(
            [&] {
                rider::validate_planar_support_model(stepper.model(),
                                                     stepper.data(), bad_high);
            },
            "invalid rider support geom");
}

// test_grip_scalar_validation_oracle — only dt=inf is expressible on the
// typed signature; the k=True / c='0' marshal rows are documented.
void test_grip_scalar_validation_oracle() {
    error<std::invalid_argument>([] {
        rider::grip_step({0., 0., 0.}, {0., 0., 0.}, 1., 0.,
                         std::numeric_limits<double>::infinity());
    });
}

// ---------------------------------------------------------------------
// test_release_preserves_reference_numeric_bool_inputs[energy-limit] —
// the reference coercion: bool energy/limit arrive as 1.0/0.0.
// ---------------------------------------------------------------------
void test_release_preserves_reference_numeric_bool_inputs(
        double energy, double limit, std::string_view params) {
    auto golden = node(kFile,
                       "test_release_preserves_reference_numeric_bool_inputs",
                       std::string(params));
    const auto actual =
            rider::release_if_overloaded({1., 0., -0.}, energy, limit);
    vec3(golden, actual.force, "");
    scalar(golden, actual.loss_j, "");
    boolean(golden, actual.overloaded, "overloaded");
    golden.done();
}

} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported,
// nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        // test_box_pad_oracle[local{i}-{angle}]
        const std::array locals{
                rider::Vec3{0., 0., 0.}, rider::Vec3{.1, 0., .02},
                rider::Vec3{.2, 0., .04}, rider::Vec3{-.2, 0., -.04},
                rider::Vec3{.1 + 1e-15, 0., .02 + 1e-15},
                rider::Vec3{-0., -0., -0.},
                rider::Vec3{.125, .04 + 1e-9, .025},
                rider::Vec3{.125 + 1e-15, .04 + 1e-9 + 1e-15, .025}};
        const std::array<std::pair<double, std::string_view>, 6> box_angles{
                {{0., "0.0"},
                 {.2, "0.2"},
                 {-.7, "-0.7"},
                 {M_PI / 2., "1.5707963267948966"},
                 {M_PI, "3.141592653589793"},
                 {2.4, "2.4"}}};
        for (std::size_t i = 0; i < locals.size(); ++i)
            for (const auto &[angle, repr] : box_angles)
                test_box_pad_oracle(i, locals[i], angle, repr);
        test_box_random_and_medial_ties();
        // test_upper_face_oracle[angle]
        for (const auto &[angle, repr] :
             std::array<std::pair<double, std::string_view>, 6>{
                     {{0., "0.0"},
                      {.3, "0.3"},
                      {M_PI / 4., "0.7853981633974483"},
                      {M_PI / 2., "1.5707963267948966"},
                      {M_PI, "3.141592653589793"},
                      {-.9, "-0.9"}}})
            test_upper_face_oracle(angle, repr);
        // test_sole_height_oracle[sole_x-compression-angle]
        {
            const std::array<std::pair<double, std::string_view>, 4> sole_xs{
                    {{0., "0.0"}, {.07, "0.07"}, {.15, "0.15"}, {2., "2.0"}}};
            const std::array<std::pair<double, std::string_view>, 5>
                    compressions{{{-0.002, "-0.002"},
                                  {0., "0.0"},
                                  {0.001, "0.001"},
                                  {0.02, "0.02"},
                                  {0.2, "0.2"}}};
            const std::array<std::pair<double, std::string_view>, 6> angles{
                    {{0., "0.0"},
                     {.2, "0.2"},
                     {.9, "0.9"},
                     {M_PI / 2., "1.5707963267948966"},
                     {M_PI, "3.141592653589793"},
                     {-.4, "-0.4"}}};
            for (const auto &[sole_x, xr] : sole_xs)
                for (const auto &[compression, cr] : compressions)
                    for (const auto &[angle, ar] : angles)
                        test_sole_height_oracle(sole_x, xr, compression, cr,
                                                angle, ar);
        }
        // test_project_goal_oracle[compression-shear-angle]
        {
            const std::array<std::pair<std::pair<double, double>,
                                       std::string_view>,
                             5>
                    pairs{{{{-.01, 0.}, "-0.01-0.0"},
                           {{.001, .02}, "0.001-0.02"},
                           {{.1, 0.}, "0.1-0.0"},
                           {{.1, 3.}, "0.1-3.0"},
                           {{.0, -3.}, "0.0--3.0"}}};
            const std::array<std::pair<double, std::string_view>, 5> angles{
                    {{0., "0.0"},
                     {.4, "0.4"},
                     {M_PI / 2., "1.5707963267948966"},
                     {M_PI, "3.141592653589793"},
                     {-.8, "-0.8"}}};
            for (const auto &[pair, pr] : pairs)
                for (const auto &[angle, ar] : angles)
                    test_project_goal_oracle(pair.first, pair.second, pr,
                                             angle, ar);
        }
        // test_grip_energy_oracle[k-c-dt]
        test_grip_energy_oracle(4000., 150., .0005, "4000.0-150.0-0.0005");
        test_grip_energy_oracle(1., 0., .1, "1.0-0.0-0.1");
        test_grip_energy_oracle(1e-100, 1e-50, 1e-20, "1e-100-1e-50-1e-20");
        // test_grip_release_oracle[limit-force{i}]
        {
            const std::array forces{rider::Vec3{3., 4., 0.},
                                    rider::Vec3{-0., 0., -0.},
                                    rider::Vec3{1e308, 1e308, 1e308}};
            const std::array<std::pair<double, std::string_view>, 3> limits{
                    {{std::nextafter(5., 0.), "4.999999999999999"},
                     {5., "5.0"},
                     {std::nextafter(5., std::numeric_limits<double>::infinity()),
                      "5.000000000000001"}}};
            for (const auto &[limit, lr] : limits)
                for (std::size_t i = 0; i < forces.size(); ++i)
                    test_grip_release_oracle(forces[i], i, limit, lr);
        }
        test_three_coordinate_hypot_oracle();
        test_invalid_grip();
        test_grip_overflow();
        test_invalid_box();
        test_outputs_own_storage_and_convert_dtype();
        // test_planar_model_validation_oracle[joint-geom-quat] — the three
        // non-rejecting rows produce no sidecar (planar: hinge about ±y,
        // in-plane slide).
        {
            const bt::TemporaryDirectory dir("rider_math_planar");
            const std::array cases{
                    PlanarCase{R"(<joint type="hinge" axis="0 1 0"/>)", "box",
                               "1 0 0 0", false},
                    PlanarCase{R"(<joint type="hinge" axis="0 -1 0"/>)", "box",
                               "1 0 0 0", false},
                    PlanarCase{R"(<joint type="slide" axis="1 0 1"/>)", "box",
                               "1 0 0 0", false},
                    PlanarCase{R"(<joint type="hinge" axis="1 0 0"/>)", "box",
                               "1 0 0 0", true},
                    PlanarCase{R"(<joint type="slide" axis="0 1 0"/>)", "box",
                               "1 0 0 0", true},
                    PlanarCase{R"(<freejoint/>)", "box", "1 0 0 0", true},
                    PlanarCase{R"(<joint type="ball"/>)", "box", "1 0 0 0",
                               true},
                    PlanarCase{R"(<joint type="hinge" axis="0 1 0"/>)",
                               "sphere", "1 0 0 0", true},
                    PlanarCase{R"(<joint type="hinge" axis="0 1 0"/>)", "box",
                               ".9238795 .3826834 0 0", true}};
            for (const auto &test_case : cases)
                test_planar_model_validation_oracle(dir, test_case);
        }
        test_grip_scalar_validation_oracle();
        test_release_preserves_reference_numeric_bool_inputs(1., 2.,
                                                             "True-2.0");
        test_release_preserves_reference_numeric_bool_inputs(0., 1.,
                                                             "False-1.0");
        test_release_preserves_reference_numeric_bool_inputs(.2, 1.,
                                                             "0.2-True");
    });
}
