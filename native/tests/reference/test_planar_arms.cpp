// Ports all eleven source test functions in
// tests/reference/test_planar_arms.py — the two-arm reference rider
// topology: the anatomical mass split (no mass or strength created),
// the 11-joint registry, the schema-2 envelope gate, the compiled-model
// topology (two arm chains, two grip sites, no midline pair), per-hand
// Jacobians, welded-hand reactions and the initial-state round trip.
//
// _environment's 30 m flat probe track rides pinned_rig; the fixture's
// pinned attachments already carry the welded grip this file measures.
// env.sim.physical's attachment_samples/attachment_errors latch is the
// period-close publication (period_buffer.py:299-305): the native twin
// is runtime::evaluate_attachments over the closing RawStep's own
// attachment_raw blocks — the same decompose_wrench projection the
// oracle's evaluate_attachments applies to its last period's raws.
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "rider_control_reference_support.hpp"
#include "configuration/rider_spec.hpp"
#include "initialization/state_file.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "model_access.hpp"
#include "rider/least_squares.hpp"
#include "runtime/period_buffer.hpp"
#include "runtime/samples.hpp"
#include "terrain/trackfile.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {
namespace rr = rider_control_reference;
namespace cfg = bike::configuration;
namespace mdl = bike::model;
namespace rcfg = ::runtime;
namespace io = bike::io;
namespace sw = ::runtime::sample_wire;
namespace init = bike::initialization;

const std::filesystem::path kEnvelopePath = rr::repo_file( // NOLINT(bugprone-throwing-static-initialization,cert-err58-cpp) static fixture table built once at startup; a throw is a fatal defect
    "examples/research/rider_joint_envelope_anatomical.json");

// _environment's flat probe track (test_planar_arms.py:130-133).
bike::terrain::TrackSpec probe_track_30() {
    return bike::terrain::track_from_dict(bike::terrain::parse_track_toml(
        "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
        "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n"));
}

double mass_at(const mdl::SegmentMasses &masses, std::string_view name) {
    for (const auto &[key, value] : masses)
        if (key == name) return value;
    bt::require(false, "segment present: " + std::string(name));
    std::unreachable();
}

std::set<std::string> mass_names(const mdl::SegmentMasses &masses) {
    std::set<std::string> out;
    for (const auto &[name, value] : masses) out.insert(name);
    return out;
}

double total_mass(const mdl::SegmentMasses &masses) {
    double total = 0.;
    for (const auto &[name, value] : masses) total += value;
    return total;
}

std::string read_text(const std::filesystem::path &path) {
  std::ifstream const stream(path);
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

const io::Value &member(const io::Value &value, std::string_view key) {
    const auto &object = std::get<io::Value::Object>(value.data);
    for (const auto &[k, v] : object)
        if (k == key) return v;
    bt::require(false, "payload member present: " + std::string(key));
    std::unreachable();
}

// The tempfile boundary is part of the loader contract (the loader reads
// paths); identical to the sibling batch files' write_payload.
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

[[gnu::noinline]] void test_split_preserves_the_anatomical_budget() {
    // tests/reference/test_planar_arms.py::test_split_preserves_the_anatomical_budget
    const auto paired = mdl::segment_masses(80., .5);
    const auto split = mdl::split_paired_arm_masses(paired);
    std::set<std::string> expected;
    for (const auto &[name, mass] : paired)
        if (name != "upper_arm_pair" && name != "forearm_pair")
            expected.insert(name);
    for (const std::string_view name :
         {"upper_arm_left", "upper_arm_right", "forearm_left",
          "forearm_right"})
        expected.emplace(name);
    bt::require(mass_names(split) == expected,
                "the split key set is the paired set minus the *_pair "
                "keys plus the four per-side segments");
    bt::require(rr::approx(total_mass(split), total_mass(paired), 1e-6,
                           1e-12),
                "the split preserves total mass");
    for (const std::string part : {"upper_arm", "forearm"})
        for (const std::string side : {"left", "right"})
            bt::require(
                rr::approx(mass_at(split, part + "_" + side), // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
                           mass_at(paired, part + "_pair") / 2.),
                part + " " + side + ": exactly half the paired segment"); // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
}

[[gnu::noinline]] void test_arm_split_does_not_add_mass_or_strength() {
    // tests/reference/test_planar_arms.py::test_arm_split_does_not_add_mass_or_strength
    const auto old_masses = mdl::segment_masses(80., 0.);
    const auto new_masses = mdl::split_paired_arm_masses(old_masses);
    bt::require(rr::approx(total_mass(new_masses), 80., 1e-9, 1e-10),
                "no mass is created (math.isclose abs_tol=1e-10)");
    for (const std::string part : {"upper_arm", "forearm"})
        bt::require(
            rr::approx(mass_at(new_masses, part + "_left") +
                           mass_at(new_masses, part + "_right"),
                       mass_at(old_masses, part + "_pair"), 1e-9, 1e-10),
            part + ": the two sides sum to the paired segment");
    // reference_joint_names() — the 11-joint registry; the shared
    // shoulder DOF is gone.
    bt::require(cfg::kRiderJoints.size() == 11,
                "the registry holds eleven internal joints");
    const std::set<std::string_view> names(cfg::kRiderJoints.begin(),
                                           cfg::kRiderJoints.end());
    bt::require(!names.contains("rider_shoulder") &&
                    names.contains("rider_shoulder_left") &&
                    names.contains("rider_shoulder_right"),
                "the paired shoulder is replaced by two per-side joints");
}

[[gnu::noinline]] void test_split_preserves_com_and_sagittal_inertia() {
    // tests/reference/test_planar_arms.py::test_split_preserves_com_and_sagittal_inertia
    cfg::RiderSpecs rider;
    rider.variant = "articulated_planar";
    cfg::detail::post_init(rider);  // RiderSpecs(variant=...) construction
    const auto props = mdl::compute_rider_centers_of_mass(rider);
    std::vector<const mdl::RiderCenter *> arms;
    for (const auto &entry : props)
        if (entry.name.find("arm") != std::string::npos)
            arms.push_back(&entry);
    const std::set<std::string> expected{
        "rider_upper_arm_left", "rider_forearm_left",
        "rider_upper_arm_right", "rider_forearm_right"};
    std::set<std::string> arm_names;
    for (const auto *entry : arms) arm_names.insert(entry->name);
    bt::require(arm_names == expected,
                "the arm centers are the four per-side segments");
    double mass = 0., com_x = 0., com_z = 0., inertia = 0.;
    for (const auto *entry : arms) {
        mass += entry->mass_kg;
        com_x += entry->mass_kg * entry->pos_m[0];
        com_z += entry->mass_kg * entry->pos_m[2];
        inertia += entry->mass_kg * (entry->pos_m[0] * entry->pos_m[0] +
                                     entry->pos_m[2] * entry->pos_m[2]);
    }
    com_x /= mass;
    com_z /= mass;
    const auto paired =
        mdl::segment_masses(rider.mass_kg, rider.helmet_mass_kg);
    const double pair_mass = mass_at(paired, "upper_arm_pair") +
                             mass_at(paired, "forearm_pair");
    bt::require(rr::approx(mass, pair_mass, 1e-12),
                "the split carries the paired arm mass (rel=1e-12)");
    for (const auto *entry : arms) {
        const double side =
            entry->name.ends_with("_left") ? -1. : 1.;
        bt::require(
            rr::approx(entry->pos_m[1], side * mdl::ARM_LATERAL_OFFSET_M),
            entry->name + ": each hand rides its own bar half-width");
    }
    bt::require(std::isfinite(com_x) && std::isfinite(com_z) &&
                    inertia > 0.,
                "the paired COM and sagittal inertia are preserved");
}

[[gnu::noinline]] void test_hand_mass_stays_inside_each_forearm() {
    // tests/reference/test_planar_arms.py::test_hand_mass_stays_inside_each_forearm
    const auto paired = mdl::segment_masses(80., .5);
    const auto split = mdl::split_paired_arm_masses(paired);
    const double body = 80. - .5;  // the helmet's mass lives in the head
    const double share = (.0162 + .0061) * body;
    for (const std::string side : {"left", "right"})
        bt::require(rr::approx(mass_at(split, "forearm_" + side), share),
                    "forearm_" + side +
                        " carries (forearm + hand) * body_mass, halved");
}

[[gnu::noinline]] void test_registry_names_eleven_internal_joints() {
    // tests/reference/test_planar_arms.py::test_registry_names_eleven_internal_joints
    // reference_joint_names() and RIDER_JOINTS are the same ordered
    // declaration; cfg::kRiderJoints backs both in one native constant.
    const std::array<std::string_view, 11> expected{
        "rider_torso_hinge",
        "rider_shoulder_left", "rider_elbow_left",
        "rider_shoulder_right", "rider_elbow_right",
        "rider_hip_front", "rider_knee_front", "rider_ankle_front",
        "rider_hip_rear", "rider_knee_rear", "rider_ankle_rear"};
    bt::require(std::ranges::equal(cfg::kRiderJoints, expected),
                "the registry is the eleven two-arm joints in order");
    for (const std::string_view name : cfg::kRiderJoints)
        bt::require(!name.starts_with("rider_root"),
                    std::string(name) + ": no root joint is internal");
}

[[gnu::noinline]] void
test_envelope_schema_two_requires_the_two_arm_topology() {
    // tests/reference/test_planar_arms.py::test_envelope_schema_two_requires_the_two_arm_topology
    const auto envelopes = cfg::load_joint_envelopes(kEnvelopePath);
    std::set<std::string> names;
    for (const auto &[name, envelope] : envelopes) names.insert(name);
    const std::set<std::string> registry(cfg::kRiderJoints.begin(),
                                         cfg::kRiderJoints.end());
    bt::require(names == registry,
                "the envelope covers exactly the registry");
    for (const std::string side : {"left", "right"})
        for (const std::string joint : {"shoulder", "elbow"}) {
            const std::string name =
                "rider_" + joint + "_" + side; // NOLINT(performance-inefficient-string-concatenation) golden label assembly mirrors the Python oracle's label building
            const auto *envelope = static_cast<const cfg::JointEnvelope *>(
                nullptr);
            for (const auto &[n, e] : envelopes)
                if (n == name) envelope = &e;
            bt::require(envelope != nullptr, name + " present");
            bt::require(envelope->minimum_anatomical_rad <
                                envelope->neutral_anatomical_rad &&
                            envelope->neutral_anatomical_rad <
                                envelope->maximum_anatomical_rad,
                        name + ": minimum < neutral < maximum");
        }
    // payload['topology'] = 'articulated_planar' — the schema-2 profile
    // must refuse a foreign topology verbatim.
    io::Value payload = io::parse_json(read_text(kEnvelopePath));
    member(payload, "topology") =
        io::Value{std::string("articulated_planar")};
    bt::TemporaryDirectory const dir("planar-arms-topology");
    const auto path = write_payload(payload, dir, "bad_topology.json");
    rr::expect_invalid(
        [&] { (void)cfg::load_joint_envelopes(path); },
        "different rider topology");
}

[[gnu::noinline]] void test_envelope_schema_missing_arm_fails_closed() {
    // tests/reference/test_planar_arms.py::test_envelope_schema_missing_arm_fails_closed
    io::Value payload = io::parse_json(read_text(kEnvelopePath));
    auto &joints = std::get<io::Value::Object>(
        member(payload, "joints").data);
    std::erase_if(joints, [](const auto &entry) {
        return entry.first == "rider_shoulder_left";
    });
    bt::TemporaryDirectory const dir("planar-arms-missing");
    const auto path = write_payload(payload, dir, "missing_arm.json");
    rr::expect_invalid(
        [&] { (void)cfg::load_joint_envelopes(path); },
        "internal rider joints");
}

[[gnu::noinline]] void
test_compiled_topology_has_two_arms_and_two_grip_sites() {
    // tests/reference/test_planar_arms.py::test_compiled_topology_has_two_arms_and_two_grip_sites
    auto rig = rr::pinned_rig({}, probe_track_30());
    const mjModel *m = rig->stepper->model();
    const mjData *d = rig->stepper->data();
    const auto eq_obj1 = model_access::readonly_buffer(
        m->eq_obj1id, m->neq, "eq_obj1id");
    const auto eq_obj2 = model_access::readonly_buffer(
        m->eq_obj2id, m->neq, "eq_obj2id");
    const auto site_xpos = model_access::readonly_buffer(
        d->site_xpos, 3 * m->nsite, "site_xpos");
    for (const auto &[side, sign] :
         std::array{std::pair{"left", -1.}, std::pair{"right", 1.}}) {
        for (const std::string part : {"upper_arm", "forearm"}) {
            const std::string body =
                "rider_" + part + "_" + side;
            bt::require(mj_name2id(m, mjOBJ_BODY, body.c_str()) >= 0,
                        body + ": the per-side body exists");
        }
        const std::string site_name = "site_rider_grip_" + std::string(side);
        const int site =
            mj_name2id(m, mjOBJ_SITE, site_name.c_str());
        bt::require(site >= 0, site_name + ": the grip site exists");
        const std::string eq_name = "connect_grip_" + std::string(side);
        const int eq =
            mj_name2id(m, mjOBJ_EQUALITY, eq_name.c_str());
        bt::require(eq >= 0, eq_name + ": the connect equality exists");
        bt::require(
            eq_obj1[static_cast<std::size_t>(eq)] ==
                mj_name2id(m, mjOBJ_BODY,
                           ("rider_forearm_" + std::string(side)).c_str()),
            eq_name + ": obj1 is that side's forearm");
        bt::require(eq_obj2[static_cast<std::size_t>(eq)] ==
                        mj_name2id(m, mjOBJ_BODY, "steer"),
                    eq_name + ": obj2 is the steer body");
        for (const std::string joint : {"shoulder", "elbow"}) {
            const std::string name =
                "rider_" + joint + "_" + side;
            bt::require(mj_name2id(m, mjOBJ_JOINT, name.c_str()) >= 0,
                        name + ": the hinge exists");
            bt::require(mj_name2id(m, mjOBJ_ACTUATOR,
                                   ("act_" + name).c_str()) >= 0,
                        name + ": the muscle actuator exists");
        }
        // Each hand grasps its own half-width point on the bar.
        bt::require(
            rr::approx(site_xpos[3 * static_cast<std::size_t>(site) + 1],
                       sign * mdl::ARM_LATERAL_OFFSET_M, 0., .01),
            site_name + ": the hand sits at its own lateral offset");
    }
    // The midline pair bodies and single grip point are gone.
    for (const char *name : {"rider_upper_arm_pair", "rider_forearm_pair"})
        bt::require(mj_name2id(m, mjOBJ_BODY, name) < 0,
                    std::string(name) + ": the paired body is gone");
    bt::require(mj_name2id(m, mjOBJ_SITE, "site_rider_grip") < 0,
                "the midline grip site is gone");
    // The spindle plant compiles nine of the eleven registry hinges (the
    // ankles ride the connect_foot_* equalities); no actuator drives a
    // root coordinate. The oracle also computes the root dof set; the
    // assert reads joint ids.
    for (const std::string_view name : cfg::kRiderJoints)
        if (name.find("ankle") == std::string_view::npos)
            bt::require(mj_name2id(m, mjOBJ_JOINT,
                                   std::string(name).c_str()) >= 0,
                        std::string(name) + ": the hinge is compiled");
    std::set<int> root_joints;
    for (const char *name :
         {"rider_root_x", "rider_root_z", "rider_root_pitch"}) {
        const int joint = mj_name2id(m, mjOBJ_JOINT, name);
        bt::require(joint >= 0, std::string(name) + " exists");
        root_joints.insert(joint);
    }
    const auto trnid = model_access::readonly_buffer(
        m->actuator_trnid, 2 * m->nu, "actuator_trnid");
    for (int a = 0; a < m->nu; ++a)
        bt::require(!root_joints.contains(
                        trnid[static_cast<std::size_t>(2) *
                              static_cast<std::size_t>(a)]),
                    "no actuator drives a root coordinate");
}

[[gnu::noinline]] void test_each_hand_jacobian_loads_only_its_own_arm() {
    // tests/reference/test_planar_arms.py::test_each_hand_jacobian_loads_only_its_own_arm
    auto rig = rr::pinned_rig({}, probe_track_30());
    mjModel *m = rig->stepper->model();
    mjData const *d = rig->stepper->data();
    const auto cross_y = [](const std::array<double, 3> &r,
                            const std::array<double, 3> &f) {
        return r[2] * f[0] - r[0] * f[2];
    };
    for (const auto &[side, other] :
         std::array{std::pair{"left", "right"},
                    std::pair{"right", "left"}}) {
        const int forearm = mj_name2id(
            m, mjOBJ_BODY, ("rider_forearm_" + std::string(side)).c_str());
        const int upper = mj_name2id(
            m, mjOBJ_BODY, ("rider_upper_arm_" + std::string(side)).c_str());
        const int site = mj_name2id(
            m, mjOBJ_SITE, ("site_rider_grip_" + std::string(side)).c_str());
        bt::require(forearm >= 0 && upper >= 0 && site >= 0,
                    "the side's chain resolves");
        const auto site_xpos = model_access::readonly_buffer(
            d->site_xpos, 3 * m->nsite, "site_xpos");
        const auto xpos = model_access::readonly_buffer(
            d->xpos, 3 * m->nbody, "xpos");
        const auto dofadr = model_access::readonly_buffer(
            m->jnt_dofadr, m->njnt, "jnt_dofadr");
        const auto point = site_xpos.subspan(
            3 * static_cast<std::size_t>(site), 3);
        std::vector<mjtNum> jac(3 * static_cast<std::size_t>(m->nv), 0.);
        mj_jac(m, d, jac.data(), nullptr, point.data(), forearm);
        const std::array<double, 3> force{0., 0., -100.};
        const auto torque = [&](int dof) {
            const auto at = [&](int row) {
                return jac[static_cast<std::size_t>(row) *
                               static_cast<std::size_t>(m->nv) +
                           static_cast<std::size_t>(dof)];
            };
            return at(0) * force[0] + at(1) * force[1] +
                   at(2) * force[2];
        };
        const auto joint_dof = [&](const char *name) {
            const int joint = mj_name2id(m, mjOBJ_JOINT, name);
            bt::require(joint >= 0,
                        std::string(name) + " resolves");
            return dofadr[static_cast<std::size_t>(joint)];
        };
        const auto lever = [&](int body) {
            const auto b = static_cast<std::size_t>(body);
            return std::array<double, 3>{
                point[0] - xpos[3 * b],
                point[1] - xpos[3 * b + 1],
                point[2] - xpos[3 * b + 2]};
        };
        // tau_elbow = (r x F)_y with r the elbow->site lever; the
        // shoulder carries the moment of the whole arm.
        const int elbow = joint_dof(
            ("rider_elbow_" + std::string(side)).c_str());
        const int shoulder = joint_dof(
            ("rider_shoulder_" + std::string(side)).c_str());
        bt::require(rr::approx(torque(elbow), cross_y(lever(forearm), force),
                               1e-9),
                    "the elbow torque is the elbow-lever moment");
        bt::require(
            rr::approx(torque(shoulder), cross_y(lever(upper), force),
                       1e-9),
            "the shoulder sees the same force through the whole-arm "
            "lever");
        // The opposite arm's actuator columns are identically zero —
        // contact ownership is not shared.
        for (const std::string joint : {"shoulder", "elbow"}) {
            const int dof = joint_dof(
                ("rider_" + joint + "_" + std::string(other)).c_str());
            bt::require(rr::approx(torque(dof), 0., 0., 1e-12),
                        "the other arm's " + joint +
                            " column is identically zero");
        }
    }
}

[[gnu::noinline]] void
test_welded_hands_report_finite_per_hand_reactions() {
    // tests/reference/test_planar_arms.py::test_welded_hands_report_finite_per_hand_reactions
    auto rig = rr::pinned_rig({}, probe_track_30());
    auto &physical = *rig->physical;
    const mjModel *m = rig->stepper->model();
    rcfg::RideControl control;
    control.motor_torque_nm = 0.;
    control.human_torque_nm = 0.;
    // env.step(control, front=1., rear=1.) forty times — one advance per
    // physical control-clock tick inside each control window.
    const auto per = std::llround(
        bike::research::ExperimentConfig{}.control_period_s /
        physical.control_clock().timestep_s);
    rcfg::RawStep last;
    for (std::int64_t i = 0; i < 40 * per; ++i)
        last = physical.advance_physics(1., 1., std::nullopt, control);
    // physical.attachment_errors == (): the closing interval's captured
    // errors plus a ':unobservable_attachment_wrench' for every raw name
    // that failed to project (period_buffer.py:300-301).
    bt::require(last.attachment_errors.empty(),
                "no captured attachment error");
    rider::LeastSquaresWorkspace workspace(
        static_cast<rider::lapack_int>(m->nv), 6, 1);
    const auto samples = rcfg::evaluate_attachments(last, workspace);
    for (const auto &[name, raw] : last.attachment_raw) {
        (void)raw;
        bt::require(
            std::ranges::any_of(samples, [&](const auto &entry) {
                return entry.first == name;
            }),
            name + ": the wrench stays observable");
    }
    const auto sample_of = [&](std::string_view name) {
        for (const auto &[entry_name, sample] : samples)
            if (entry_name == name) return &sample;
        bt::require(false, "grip sample present: " + std::string(name));
        std::unreachable();
    };
    for (const std::string side : {"left", "right"}) {
        const auto &sample = *sample_of("grip_" + side);
        bt::require(sample.kind == "grip", "grip_" + side + ": kind");
        bt::require(std::isfinite(sample.normal_n) &&
                        std::isfinite(sample.tangent_n),
                    "grip_" + side + ": finite normal and tangent");
        bt::require(std::isfinite(sample.gap_m) && sample.gap_m < .005,
                    "grip_" + side + ": the weld gap is below 5 mm");
    }
    // physical.sample.channels['rider_welds'] — accounting folds
    // diagnostics.rider_welds into the completed sample's channels
    // verbatim; the closing RawStep's diagnostics are the same object.
    const auto &welds =
        sw::object(sw::required(last.diagnostics, "rider_welds"));
    const auto force_on_rider = [&](const char *name) {
        const auto &entry = sw::object(sw::required(welds, name));
        const auto &vec =
            sw::array(sw::required(entry, "force_on_rider_n"));
        bt::require(vec.size() == 3, "force_on_rider_n is a vec3");
        return std::array<double, 3>{sw::number(vec[0]),
                                     sw::number(vec[1]),
                                     sw::number(vec[2])};
    };
    const auto left = force_on_rider("grip_left");
    const auto right = force_on_rider("grip_right");
    for (const double value : {left[0], left[1], left[2], right[0],
                               right[1], right[2]})
        bt::require(std::isfinite(value),
                    "per-hand reactions stay finite");
    // Symmetric pose, symmetric road: the two hands split the bar load
    // evenly — in-plane components agree, the out-of-plane residual is
    // antisymmetric and negligible.
    rr::allclose(std::array{left[0], left[2]},
                 std::array{right[0], right[2]}, .01, 0.,
                 "the in-plane hand loads agree (rtol=1e-2)");
    bt::require(rr::approx(left[1], -right[1], 1e-7, 1e-6),
                "the out-of-plane residual is antisymmetric");
}

[[gnu::noinline]] void
test_initial_state_round_trip_restores_per_hand_anchors() {
    // tests/reference/test_planar_arms.py::test_initial_state_round_trip_restores_per_hand_anchors
    auto rig = rr::pinned_rig({}, probe_track_30());
    auto &context = (rig->context).value(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
    // PhysicalInitialState.capture(env.sim) — only at t=0, like the
    // oracle's _guard.
    const auto state = init::PhysicalInitialState::capture(context);
    const auto payload = state.payload();
    // state.payload['grip_anchor_local'] == {'left', 'right'} — a vec3
    // per hand.
    const auto &anchors = io::as_object(
        member(payload, "grip_anchor_local"), "grip_anchor_local");
    bt::require(io::find_member(anchors, "left") != nullptr &&
                    io::find_member(anchors, "right") != nullptr &&
                    anchors.size() == 2,
                "the payload carries exactly the two per-hand anchors");
    std::array<std::array<double, 3>, 2> expected{};
    for (std::size_t side = 0; side < 2; ++side) {
        const char *name = side == 0 ? "left" : "right";
        const auto &vec = io::as_array(
            *io::find_member(anchors, name), "grip anchor");
        bt::require(vec.size() == 3, "the anchor is a vec3");
        for (std::size_t axis = 0; axis < 3; ++axis)
            expected[side][axis] = io::as_double(vec[axis], "anchor");
    }
    // env.sim.physical_initial_state = state; env.sim.physical.reset():
    // reset() sets initializing=True and runs the seeded branch
    // (seed.prepare) — resetData + material/anchor restore +
    // apply_forces — exactly PhysicalInitialState::prepare.
    context.initializing = true;
    context.equilibrium = state.prepare(context);
    context.initializing = false;
    const auto &restored =
        rig->stepper->rider_contacts().state().grip_anchor_local;
    for (std::size_t side = 0; side < 2; ++side) {
        bt::require(restored[side].has_value(),
                    std::string(side == 0 ? "left" : "right") +
                        ": the anchor is restored");
        for (std::size_t axis = 0; axis < 3; ++axis)
            bt::require(
                rr::approx(((restored[side]).value())[axis], // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture contract under test; .value() reports violations loudly
                           expected[side][axis], 1e-7, 1e-12),
                "the per-hand anchor round-trips verbatim");
    }
}
} // namespace

int main() {
    return bt::run([] {
        test_split_preserves_the_anatomical_budget();
        test_arm_split_does_not_add_mass_or_strength();
        test_split_preserves_com_and_sagittal_inertia();
        test_hand_mass_stays_inside_each_forearm();
        test_registry_names_eleven_internal_joints();
        test_envelope_schema_two_requires_the_two_arm_topology();
        test_envelope_schema_missing_arm_fails_closed();
        test_compiled_topology_has_two_arms_and_two_grip_sites();
        test_each_hand_jacobian_loads_only_its_own_arm();
        test_welded_hands_report_finite_per_hand_reactions();
        test_initial_state_round_trip_restores_per_hand_anchors();
    });
}
