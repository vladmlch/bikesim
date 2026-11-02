// Ports tests/reference/test_seated_topology.py — reference seated
// topology: a saddle pin frees pelvis pitch, platforms spin.
//
// The ElementTree cases map onto the same xml::Element document
// add_saddle_pin consumes natively; the engine cases run the same held
// pinned-plant episode the oracle's make_environment produces — the
// live welded TOML (already spindle/pin/connect under F1) on a flat
// probe track, so _pin_config's string replace is a no-op on both
// sides and only the envelope/strength path spellings differ.
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "engine_call.hpp"
#include "initialization/setup_owner.hpp"
#include "model/builder_internal.hpp"
#include "model/rider_geometry.hpp"
#include "model/xml_writer.hpp"
#include "model_access.hpp"
#include "research/environment.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/spindle_math.hpp"
#include "runtime/control.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include "runtime/step.hpp"
#include "stepper.hpp"
#include "support/golden.hpp"
#include "support/fixture_test.hpp"
#include "terrain/trackfile.hpp"
#include "validation/runtime_access.hpp"
#include "writers/rider_contacts.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mujoco/mujoco.h>
#include <numbers>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace {

namespace rider = ::rider;
namespace research = bike::research;
namespace cfg = bike::configuration;
namespace xml = bike::model::xml;

constexpr double kPi = std::numbers::pi;

// The live example document — bt::oracle::root() is
// <repo>/tests/fixtures/golden. The oracle's WELDED constant reads this
// same file; the migration fixture mirror predates F1 and lacks the
// activation lag, delivered budgets and weld envelopes the pinned-plant
// episodes assert.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

double round12(double value) { return std::round(value * 1e12) / 1e12; }

// The miniature MJCF document the oracle's ElementTree cases build:
// pelvis + saddle site, a frame body and an equality block.
xml::Element miniature(bool legacy_weld) {
    xml::Element root("mujoco");
    auto &world = root.add("worldbody");
    auto &pelvis = world.add("body", {{"name", "rider_pelvis"}});
    pelvis.add("site", {{"name", "site_rider_saddle"},
                        {"pos", "0 0 -0.060"}});
    world.add("body", {{"name", "frame"}});
    auto &equality = root.add("equality");
    if (legacy_weld)
        equality.add("weld", {{"name", "weld_saddle"},
                              {"body1", "rider_pelvis"},
                              {"body2", "frame"}});
    return root;
}

// test_saddle_connection_does_not_constrain_orientation
void test_saddle_connection_does_not_constrain_orientation() {
    auto root = miniature(false);
    bike::model::detail::add_saddle_pin(root, .0025);
    auto const *equality = root.child("equality");
    // The null guards must be visible in this translation unit:
    // bt::require's throw lives in the harness TU, so the analyzer cannot
    // prove the -> dereferences below are unreachable on the null path.
    // The failure mode is identical — std::runtime_error with the same
    // messages.
    if (equality == nullptr)
        throw std::runtime_error("the equality block exists");
    auto const *connect = equality->find("connect", "connect_saddle");
    if (connect == nullptr)
        throw std::runtime_error("the saddle pin emits a connect equality");
    bt::require(connect->get("body1") == "rider_pelvis",
                "the connect binds the pelvis");
    bt::require(connect->get("body2") == "frame",
                "the connect binds the frame");
    bt::require(connect->get("anchor") == "0 0 -0.060",
                "the anchor is the saddle site position, verbatim");
    bt::require(equality->find("weld", "weld_saddle") == nullptr,
                "no weld constrains the pelvis orientation");
}

// test_saddle_pin_replaces_a_legacy_weld_and_validates
void test_saddle_pin_replaces_a_legacy_weld_and_validates() {
    {
        auto root = miniature(true);
        bike::model::detail::add_saddle_pin(root, .0025);
        auto const *equality = root.child("equality");
        bt::require(equality->find("weld", "weld_saddle") == nullptr,
                    "the legacy weld is removed");
        bt::require(equality->find("connect", "connect_saddle") != nullptr,
                    "the connect replaces it");
    }
    {
        xml::Element bare("mujoco");
        (void)bare.add("worldbody");
        bt::invalid([&] { bike::model::detail::add_saddle_pin(bare, .0025); });
    }
    {
        auto root = miniature(false);
        bt::invalid([&] { bike::model::detail::add_saddle_pin(root, 0.); });
    }
}

// test_anatomical_angle_inverts_the_declared_range
void test_anatomical_angle_inverts_the_declared_range() {
    const cfg::JointEnvelope envelope{
        .neutral_anatomical_rad = .3, .direction = -1,
        .minimum_anatomical_rad = -.2, .maximum_anatomical_rad = 1.1,
        .provenance = "synthetic-unit-fixture"};
    const auto [lo, hi] = cfg::joint_q_range(envelope);
    // The q-bound endpoints map to the two declared anatomical extremes,
    // swapped under a negative direction convention.
    const std::set<double> mapped{round12(cfg::anatomical_angle(envelope, lo)),
                                  round12(cfg::anatomical_angle(envelope, hi))};
    bt::require(mapped == std::set<double>{round12(-.2), round12(1.1)},
                "the q endpoints map to the declared anatomical extremes");
    bt::require(std::abs(cfg::anatomical_angle(envelope, 0.) - .3) <=
                    1e-6 * .3,
                "q=0 reads the neutral angle");
    const cfg::JointEnvelope forward{
        .neutral_anatomical_rad = .3, .direction = 1,
        .minimum_anatomical_rad = -.2, .maximum_anatomical_rad = 1.1,
        .provenance = "synthetic-unit-fixture"};
    const auto [flo, fhi] = cfg::joint_q_range(forward);
    bt::require(std::abs(cfg::anatomical_angle(forward, flo) - (-.2)) <=
                    1e-6 * .2 &&
                    std::abs(cfg::anatomical_angle(forward, fhi) - 1.1) <=
                        1e-6 * 1.1,
                "a positive direction maps endpoints in order");
    // A wrapped coordinate cannot fake being inside the range.
    bt::require(std::abs(cfg::anatomical_angle(envelope, 2. * kPi) - .3) <=
                    1e-6 * .3,
                "a 2 pi wrap normalizes back to the neutral angle");
}

// test_rom_check_names_out_of_range_joints
void test_rom_check_names_out_of_range_joints() {
    cfg::JointEnvelopeMap envelopes;
    rider::NamedEntries<double> angles;
    for (const auto name : cfg::kRiderJoints) {
        envelopes.emplace_back(std::string(name),
                               cfg::JointEnvelope{
                                   .neutral_anatomical_rad = 0.,
                                   .direction = 1,
                                   .minimum_anatomical_rad = -1.,
                                   .maximum_anatomical_rad = 1.,
                                   .provenance = "synthetic-unit-fixture"});
        angles.emplace_back(std::string(name), 0.);
    }
    bt::require(cfg::reference_rom_check(angles, envelopes).empty(),
                "every neutral angle is inside its envelope");
    for (auto &[name, value] : angles)
        if (name == "rider_torso_hinge") value = 2.;
    const auto violations = cfg::reference_rom_check(angles, envelopes);
    bt::require(violations == std::vector<std::string>{"rider_torso_hinge"},
                "the violating joint is named in envelope order");
}

// ---- the held pinned-plant episode the three slow checks share --------

struct PinnedEnv {
    std::unique_ptr<runtime::NativeRideRuntime> runtime;
    std::int64_t period = 0;
};

// _environment: the live welded TOML on a flat 30 m probe track. The
// diagnostic-model-limits and 1e9 energy tolerance flags are the
// monitor-side knobs; the settle solves fresh because the captured
// initial-state contracts hash the pre-F1 fixture physics.
PinnedEnv pinned_env() {
    auto owner = std::make_unique<research::ResearchRequest>();
    auto &request = *owner;
    request.setup.model.physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    request.setup.model.mode = "ride";
    request.setup.model.crank_joint = true;
    request.setup.model.physics->equilibrium_cache_enabled = false;
    request.setup.model.rider =
        bike::model::rider_from_args("articulated_planar");
    request.setup.track = bike::terrain::track_from_dict(
        bike::terrain::parse_track_toml(
            "name = \"probe\"\nlength_m = 30.0\nsurface = \"hardpack\"\n"
            "grade_profile = { knots = [[0.0, 0.0], [30.0, 0.0]] }\n"));
    request.experiment.duration_s = 1.;
    request.experiment.record_decimation = 80;
    request.experiment.stop_on_model_violation = false;
    request.experiment.maximum_energy_residual_ratio = 1e9;
    request.sensors = research::SensorConfig{};
    request.sensors.sample_period_s = .005;
    const auto prepared = research::prepare_environment(std::move(owner));
    auto bootstrap = std::make_unique<runtime::RuntimeBootstrap>(
        std::move(prepared->bootstrap.ride));
    PinnedEnv env;
    // One env.step(control): the research control window (.01 s / dt).
    env.period = prepared->bootstrap.config.control_steps;
    env.runtime =
        std::make_unique<runtime::NativeRideRuntime>(std::move(*bootstrap));
    return env;
}

const rider::AttachmentSample *attachment_sample(
    const Stepper &stepper, std::string_view name) {
    const auto &state = stepper.rider_contacts().state();
    for (const auto &[key, entry] : state.attachment_samples)
        if (key == name)
            if (const auto *sample =
                    std::get_if<rider::AttachmentSample>(&entry))
                return sample;
    return nullptr;
}

// test_pin_rows_free_pelvis_pitch_and_carry_no_moment (slow)
void test_pin_rows_free_pelvis_pitch_and_carry_no_moment() {
    auto env = pinned_env();
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [](const Stepper &stepper,
                         const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            const int eq = mj_name2id(m, mjOBJ_EQUALITY, "connect_saddle");
            bt::require(eq >= 0, "the saddle pin compiles to connect_saddle");
            bt::require(model_access::readonly_buffer(m->eq_type, m->neq)
                            [static_cast<std::size_t>(eq)] == mjEQ_CONNECT,
                        "the saddle equality is a CONNECT");
            bt::require(mj_name2id(m, mjOBJ_EQUALITY, "weld_saddle") < 0,
                        "no weld_saddle remains in the compiled plant");
            const int pitch =
                mj_name2id(m, mjOBJ_JOINT, "rider_root_pitch");
            bt::require(pitch >= 0,
                        "the pelvis pitch degree of freedom exists");
            // No actuator may write to any rider root degree of freedom.
            std::set<int> root_joints;
            for (const char *name :
                 {"rider_root_x", "rider_root_z", "rider_root_pitch"})
                root_joints.insert(mj_name2id(m, mjOBJ_JOINT, name));
            const auto trnid = model_access::readonly_buffer(
                m->actuator_trnid, 2 * m->nu);
            for (int a = 0; a < m->nu; ++a)
                bt::require(
                    root_joints.count(trnid[2 * static_cast<std::size_t>(a)]) ==
                        0,
                    "no actuator writes to a rider root dof");
        });
    const auto result = env.runtime->advance(
        env.runtime->snapshot().step + env.period,
        {.motor_torque_nm = 0., .motor_limit_nm = std::nullopt,
         .human_torque_nm = 0.,
         .crank_target_rate_rad_s = std::nullopt,
         .posture = std::nullopt}, 1., 1.,
        std::nullopt);
    bt::require(result.reason != "outcome",
                "the held pinned plant survives a control tick");
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [](const Stepper &stepper,
                         const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            const int eq = mj_name2id(m, mjOBJ_EQUALITY, "connect_saddle");
            rider::EqualityReactions const reactions(m);
            const auto rows = reactions.equality_rows(stepper.data());
            const auto found =
                std::ranges::find(rows, eq, &decltype(rows)::value_type::first);
            bt::require(found != rows.end() && found->second.size() == 3,
                        "the saddle connect carries exactly three rows");
            const auto *sample = attachment_sample(stepper, "saddle");
            bt::require(sample != nullptr,
                        "the saddle attachment sample is published");
            bt::require(sample->moment_nm == 0.,
                        "a connect carries no moment");
            bt::require(sample->gap_m <= .005,
                        "the saddle gap stays inside the link bound");
            bt::require(std::isfinite(sample->normal_n) &&
                            std::isfinite(sample->tangent_n),
                        "the solved normal and tangent loads are finite");
        });
}

// test_pedal_platform_keeps_its_spin_joint (slow)
void test_pedal_platform_keeps_its_spin_joint() {
    auto env = pinned_env();
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [](const Stepper &stepper,
                         const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            for (const char *side : {"front", "rear"}) {
                const std::string suffix = side;
                const int joint = mj_name2id(
                    m, mjOBJ_JOINT, ("pedal_" + suffix + "_spin").c_str());
                bt::require(joint >= 0,
                            "the pedal platform retains its spin joint");
                const auto jnt_type = model_access::readonly_buffer(
                    m->jnt_type, m->njnt);
                bt::require(
                    jnt_type[static_cast<std::size_t>(joint)] == mjJNT_HINGE,
                    "the pedal spin is a hinge");
                bt::require(model_access::readonly_buffer(m->jnt_limited,
                                                        m->njnt)
                                [static_cast<std::size_t>(joint)] == 0,
                            "the pedal spin is unlimited");
                const int connect = mj_name2id(
                    m, mjOBJ_EQUALITY,
                    ("connect_foot_" + suffix).c_str());
                bt::require(
                    connect >= 0 &&
                        model_access::readonly_buffer(m->eq_type, m->neq)
                            [static_cast<std::size_t>(connect)] == mjEQ_CONNECT,
                    "the foot attaches through a connect");
                // The foot connects to the free-spinning spindle, never to
                // the crank arm or the frame.
                const int body2 = model_access::readonly_buffer(
                    m->eq_obj2id, m->neq)[static_cast<std::size_t>(connect)];
                bt::require(body2 == mj_name2id(m, mjOBJ_BODY,
                                                ("pedal_" + suffix).c_str()),
                            "the foot connect targets the pedal body");
            }
        });
}

// test_built_pose_anatomical_angles_inside_envelope (slow)
void test_built_pose_anatomical_angles_inside_envelope() {
    auto env = pinned_env();
    bike::validation::RuntimeRigAccess::inspect(
        *env.runtime, [](const Stepper &stepper,
                         const runtime::PhysicalStep &) {
            const mjModel *m = stepper.model();
            const mjData *d = stepper.data();
            std::vector<std::string_view> present;
            for (const auto name : cfg::kRiderJoints)
                if (mj_name2id(m, mjOBJ_JOINT, std::string(name).c_str()) >=
                    0)
                    present.push_back(name);
            const auto envelopes = cfg::load_joint_envelopes(
                repo_file("examples/research/"
                          "rider_joint_envelope_anatomical.json"),
                std::span<const std::string_view>(present));
            rider::NamedEntries<double> angles;
            for (const auto name : present) {
                const int joint =
                    mj_name2id(m, mjOBJ_JOINT, std::string(name).c_str());
                bt::require(joint >= 0, "the present joint resolves");
                const int qpos_adr = model_access::readonly_buffer(
                    m->jnt_qposadr, m->njnt)
                                         [static_cast<std::size_t>(joint)];
                angles.emplace_back(
                    std::string(name),
                    model_access::readonly_buffer(d->qpos, m->nq)
                        [static_cast<std::size_t>(qpos_adr)]);
            }
            bt::require(
                cfg::reference_rom_check(angles, envelopes).empty(),
                "the built pose passes the ROM check");
            // Every joint's assembled-pose angle is a real anatomical
            // read, not a round-trip of its own declared range.
            for (const auto &[name, envelope] : envelopes) {
                const auto found = std::ranges::find(
                    angles, name, &decltype(angles)::value_type::first);
                bt::require(found != angles.end(),
                            "the angle table carries the joint");
                const double angle =
                    cfg::anatomical_angle(envelope, found->second);
                bt::require(envelope.minimum_anatomical_rad <= angle &&
                                angle <= envelope.maximum_anatomical_rad,
                            "the pose angle lies inside the declared "
                            "anatomical envelope");
            }
        });
}

} // namespace

int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        test_saddle_connection_does_not_constrain_orientation();
        test_saddle_pin_replaces_a_legacy_weld_and_validates();
        test_anatomical_angle_inverts_the_declared_range();
        test_rom_check_names_out_of_range_joints();
        test_pin_rows_free_pelvis_pitch_and_carry_no_moment();
        test_pedal_platform_keeps_its_spin_joint();
        test_built_pose_anatomical_angles_inside_envelope();
    });
}
