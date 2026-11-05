// Ports tests/reference/test_pinned_topology.py — the seated spindle
// plant keeps nine hinges and force-only attachments.
//
// _pinned_config's regex rewrites are no-ops on the F1 live document —
// it already declares spindle/pin/connect — and only the envelope/
// strength path spellings differ. _compiled() maps onto the same
// BuildRequest assembly the runtime projects: resolve_rider on the
// articulated_planar variant, crank_joint, then generate_mjcf +
// compile_mjcf like generate_mujoco_xml, with the controller built from
// the runtime's own projected rider_controller document.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "diag.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "configuration/rider_spec.hpp"
#include "engine_call.hpp"
#include "geometry/specs.hpp"
#include "io/json.hpp"
#include "model_access.hpp"
#include "model/builder.hpp"
#include "model/rider_geometry.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/state_codec.hpp"
#include "runtime/project_config.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace {

namespace rider = ::rider;
namespace cfg = bike::configuration;
namespace bio = bike::io;
namespace mdl = bike::model;
namespace geo = bike::geometry;

// The repository root — bt::oracle::root() is <repo>/tests/fixtures/golden.
// The oracle resolves the two profile documents next to the live welded
// TOML under examples/research/.
std::filesystem::path repo_file(std::string_view relative) {
    return bt::oracle::root().parent_path().parent_path().parent_path() /
           std::filesystem::path(relative);
}

const std::filesystem::path envelope_file() {
    return repo_file("examples/research/rider_joint_envelope_anatomical.json");
}
const std::filesystem::path strength_file() {
    return repo_file("examples/research/rider_strength_reference.json");
}

// _compiled(): compile the pinned plant and build the spindle
// controller exactly like the oracle's ArticulatedRiderController.
struct Compiled {
    mdl::ModelOwner model;
    rider::SpindlePose pose;
    rider::SpindleConfig config;
};

Compiled compiled() {
    const auto physics = cfg::load_physics(
        repo_file("examples/research/viewer_physics_welded.toml"));
    mdl::BuildRequest request;
    request.mode = "ride";
    request.include_rider = true;
    // generate_mujoco_xml(specs, mode='ride', rider=RiderSpecs(
    // variant='articulated_planar'), crank_joint=True, physics_config=cfg)
    const auto rider = mdl::resolve_rider(
        cfg::RiderSpecs{.variant = "articulated_planar",
                        .inseam_m = std::nullopt},
        true, "lumped");
    request.rider = rider;
    request.crank_joint = true;
    request.physics = physics;
    Compiled out;
    out.model =
        mdl::compile_mjcf(mdl::generate_mjcf(request));
    // The runtime's own projection produces the controller wire —
    // strength and envelope tables resolve from the config paths like
    // the oracle's in-__init__ loads.
    out.pose = rider::decode_spindle_pose(
        runtime::project_articulated_pose(
            mdl::geometry_pose(rider, request.bike)),
        "pinned.pose");
    out.config = rider::decode_spindle_config(
        runtime::project_rider_controller(physics, *out.model));
    return out;
}

std::set<std::string> equality_names(const mjModel &m) {
    std::set<std::string> names;
    for (int i = 0; i < m.neq; ++i)
        if (const char *name = mj_id2name(&m, mjOBJ_EQUALITY, i))
            names.insert(name);
    return names;
}

// test_pinned_plant_topology (slow)
void test_pinned_plant_topology() {
    auto plant = compiled();
    const mjModel &m = *plant.model;
    const auto names = equality_names(m);
    const auto eq_types =
        model_access::readonly_buffer(m.eq_type, m.neq);
    for (const char *name :
         {"connect_foot_front", "connect_foot_rear", "connect_saddle",
          "connect_grip_left", "connect_grip_right"}) {
        const int eq = mj_name2id(&m, mjOBJ_EQUALITY, name);
        bt::require(eq >= 0 &&
                        eq_types[static_cast<std::size_t>(eq)] ==
                            mjEQ_CONNECT,
                    "the attachment is a CONNECT equality");
    }
    bt::require(!std::ranges::any_of(names, [](const auto &name) {
        return name.starts_with("weld_");
    }), "no weld_ equalities remain on the pinned plant");
    for (const char *side : {"front", "rear"})
        bt::require(mj_name2id(&m, mjOBJ_JOINT,
                               ("rider_ankle_" + std::string(side)).c_str()) ==
                        -1,
                    "the locked ankle is absent from the pinned plant");
    const auto forcelimited = model_access::readonly_buffer(
        m.actuator_forcelimited, m.nu);
    for (int i = 0; i < m.nu; ++i)
        if (const char *name = mj_id2name(&m, mjOBJ_ACTUATOR, i);
            name != nullptr && std::string_view(name).starts_with("act_rider_"))
            bt::require(forcelimited[static_cast<std::size_t>(i)] == 0,
                        "rider actuators are not force-limited");
    // ArticulatedRiderController(model, pose, cfg.articulated, crank/1000.)
    const geo::BikeSpecs specs;
    rider::SpindleController const controller(
        &m, plant.pose, std::move(plant.config), specs.crank_length / 1000.);
    std::set<std::string> joints;
    for (const auto &entry : controller.joints())
        joints.insert(entry.name);
    bt::require(joints == std::set<std::string>{
                             "rider_torso_hinge",
                             "rider_shoulder_left", "rider_shoulder_right",
                             "rider_elbow_left", "rider_elbow_right",
                             "rider_hip_front", "rider_hip_rear",
                             "rider_knee_front", "rider_knee_rear"},
                "the active controller joints are the nine-hinge set");
    bt::require(std::set<std::string>(controller.locked_joints().begin(),
                                      controller.locked_joints().end()) ==
                    std::set<std::string>{"rider_ankle_front",
                                          "rider_ankle_rear"},
                "the locked joints are both ankles");
    const int eq = mj_name2id(&m, mjOBJ_EQUALITY, "connect_foot_front");
    const auto eq_data = model_access::readonly_buffer(
        m.eq_data, m.neq * mjNEQDATA);
    bt::require(
        std::abs(
            eq_data[static_cast<std::size_t>(eq) * mjNEQDATA + 2] - (-.115)) <=
            1e-6 * .115,
        "the front foot anchor is -0.115 m like the reference");
}

// test_joint_profiles_select_present_joints_and_require_them
void test_joint_profiles_select_present_joints_and_require_them() {
    std::vector<std::string_view> present;
    for (const auto name : cfg::kRiderJoints)
        if (!name.contains("ankle")) present.push_back(name);
    const auto envelopes = cfg::load_joint_envelopes(
        envelope_file(), std::span<const std::string_view>(present));
    std::set<std::string> names;
    for (const auto &[name, unused] : envelopes) names.insert(name);
    bt::require(names == std::set<std::string>(present.begin(), present.end()),
                "the envelope loader selects exactly the present joints");
    const auto profile =
        cfg::load_strength_profile(strength_file(), present, false);
    names.clear();
    for (const auto &[name, curves] : profile) names.insert(name);
    bt::require(names == std::set<std::string>(present.begin(), present.end()),
                "the strength loader selects exactly the present joints");
    const auto coordinates =
        cfg::load_strength_coordinates(strength_file(), present);
    names.clear();
    for (const auto &[name, envelope] : coordinates) names.insert(name);
    bt::require(names == std::set<std::string>(present.begin(), present.end()),
                "the coordinate loader selects exactly the present joints");
    // A profile that drops a present joint is rejected, for both files.
    const auto scratch =
        std::filesystem::temp_directory_path() /
        "pinned_topology_profiles";
    std::filesystem::create_directories(scratch);
    for (const auto &[file, strength] :
         {std::pair{envelope_file(), false},
          std::pair{strength_file(), true}}) {
        auto payload = bio::parse_json(
            bt::text(file));
        auto &root = std::get<bio::Value::Object>(payload.data);
        const auto joints = std::ranges::find(
            root, "joints", &bio::Value::Object::value_type::first);
        bt::require(joints != root.end(),
                    "the profile carries a joints map");
        auto &map = std::get<bio::Value::Object>(joints->second.data);
        map.erase(std::ranges::remove_if(map,
                                         [](const auto &entry) {
                                           return entry.first ==
                                                  "rider_knee_front";
                                         })
                      .begin(),
                  map.end());
        const auto path = scratch / file.filename();
        std::ofstream(path) << bio::recording_json(payload);
        if (strength)
            bt::invalid([&] {
                (void)cfg::load_strength_profile(path, present, false);
            });
        else
            bt::invalid([&] {
                (void)cfg::load_joint_envelopes(
                    path, std::span<const std::string_view>(present));
            });
    }
}

} // namespace

// The Compiled record embeds the resolved SpindleConfig wire — multi-KiB
// of automatic storage, over the -Wframe-larger-than=8192 gate like the
// sibling rider-forces port.
// cppcheck-suppress unknownMacro // NATIVE_DIAG_* is the clang-dialect _Pragma wrapper from diag.hpp; cppcheck does not resolve it through -isystem
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_pinned_plant_topology();
        test_joint_profiles_select_present_joints_and_require_them();
    });
}
