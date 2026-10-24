// test_native_rider_contacts.cpp — C++ port of
// tests/reference/test_native_rider_contacts.py (batch 3.14a, T52).
//
// Two halves, mirroring the source file: the projected rider_contacts
// section's wire schema (pure ports — the env fixture's equilibrium
// baseline keeps those pytest nodes off the golden path; see the port-map
// record and migration/baseline-refresh-2026-10-23.md), and the
// model-owned RiderContactWriter core + T3b-2 settle/sample face on the
// tiny _contact_xml plant, replaying the frozen two-platform goldens.
//
// The Python oracle owned a second (model, data) pair mirrored onto the
// Stepper per call; the port drives the Stepper's own buffers through the
// same write/forward sequence, so every assertion is single-owner.
#include <cmath>
#include <cstdint>
#include <ranges>

#include "diag.hpp"
#include "rider_contacts_reference_support.hpp"

namespace {

using namespace rider_contacts_reference;
using bike::io::Value;
namespace wc = writers;

constexpr std::string_view kFile = "test_native_rider_contacts.py";
constexpr double kDt = 0.0005; // _contact_xml's timestep="0.0005"

// rider_contacts_qfrc through the binding's mutate boundary.
[[nodiscard]] std::vector<double> qfrc(Stepper &stepper, double dt,
                                       bool advance = true,
                                       bool detailed = true) {
    return stepper.mutate([&] {
        return stepper.rider_contacts_qfrc(dt, advance, detailed);
    });
}

// set_rider_contacts_state: decode (validation, never mutates) + assign
// inside mutate — the binding's two halves verbatim.
void set_state(Stepper &stepper, const Value &document) {
    wc::RiderContactWriter &writer = stepper.rider_contacts();
    auto [state, probe] =
            rider::decode_rider_contacts_state(document, writer);
    stepper.mutate([&] {
        writer.assign_state(std::move(state), std::move(probe));
    });
}

// ---- Value-tree mutation helpers (the corrupt()-style candidates) ----

[[nodiscard]] Value::Object *object_of(Value &value) {
    return std::get_if<Value::Object>(&value.data);
}

[[nodiscard]] Value *member_of(Value &value, std::string_view key) {
    auto *const object = object_of(value);
    if (object == nullptr) return nullptr;
    for (auto &[name, child] : *object)
        if (name == key) return &child;
    return nullptr;
}

void set_key(Value &value, std::string_view key, Value child) {
    auto *const object = object_of(value);
    bt::require(object != nullptr,
                "set_key: value is not an object for '" +
                        std::string(key) + "'");
    for (auto &[name, existing] : *object)
        if (name == key) {
            existing = std::move(child);
            return;
        }
    object->emplace_back(std::string(key), std::move(child));
}

void remove_key(Value &value, std::string_view key) {
    auto *const object = object_of(value);
    bt::require(object != nullptr,
                "remove_key: value is not an object for '" +
                        std::string(key) + "'");
    std::erase_if(*object,
                  [&](const auto &entry) { return entry.first == key; });
}

[[nodiscard]] Value object_value(std::initializer_list<
                                 std::pair<std::string, Value>>
                                         members) {
    Value::Object object;
    for (auto &member : members)
        object.emplace_back(member.first, std::move(member.second));
    return Value{std::move(object)};
}

[[nodiscard]] Value bool_value(bool flag) { return Value{flag}; }
[[nodiscard]] Value null_value() { return Value{std::monostate{}}; }

// The {'schema': 2, 'rider_contacts': section} doc as a mutable tree —
// test bodies corrupt this, never the production projection.
[[nodiscard]] Value section_tree(Pedal pedal = Pedal::flat,
                                 Saddle saddle = Saddle::flat,
                                 Grip grip = Grip::spring,
                                 bool pair_limit = false,
                                 double pair_limit_n = 0.,
                                 double arm_reach = 0.447213595499958) {
    return contacts_section(pedal, saddle, grip, pair_limit, pair_limit_n,
                            arm_reach);
}

// The whole {'schema': ..., 'rider_contacts': ...} document.
[[nodiscard]] Value document_with(Value section) {
    Value::Object root;
    root.emplace_back("schema", setupvalue::integer(2));
    root.emplace_back("rider_contacts", std::move(section));
    return Value{std::move(root)};
}

// decode() on a document — the Stepper ctor's typed-config half.
[[nodiscard]] nativecfg::WriterConfig decode_doc(const Value &document) {
    return nativecfg::decode(document);
}

// ==========================================================================
// The env-fixture block: schema + projection contracts (pure ports).
// The pytest nodes bind a module fixture whose equilibrium warm start is
// the documented cold-cache baseline failure on this lane's fingerprint —
// migration/baseline-refresh-2026-10-23.md. Their checks are wire-schema
// contracts independent of that model: the port exercises them against
// the identical section shape the tiny plant projects.
// ==========================================================================

// test_projection_copies_the_appliers_resolved_setup — every projected
// key lands in the typed config with the projected value; the section is
// exactly REQUIRED ∪ OPTIONAL.
void test_projection_copies_the_appliers_resolved_setup() {
    const auto config = decode_doc(document_with(section_tree()));
    bt::require(config.rider_contacts.has_value(),
                "decoded config carries the rider_contacts section");
    const auto &c = *config.rider_contacts;
    bt::require(c.arm_reach_m == 0.447213595499958,
                "arm_reach_m is the pose-resolved reach");
    bt::require(c.saddle_patch_half_length_m == 0.045 &&
                        c.pedal_patch_half_length_m == 0.025 &&
                        c.support_pad_radius_m == 0.02 &&
                        c.support_k_n_m == 100000.0 &&
                        c.support_c_ns_m == 500.0 &&
                        c.pedal_c_ns_m == 100.0 &&
                        c.support_tangent_k_n_m == 20000.0 &&
                        c.support_mu == 0.8 && c.support_length_m == 0.1 &&
                        c.grip_k_n_m == 4000.0 && c.grip_c_ns_m == 150.0 &&
                        c.grip_release_distance_m == 0.12 &&
                        c.grip_capture_distance_m == 0.02 &&
                        c.grip_capture_speed_mps == 0.2,
                "every ArticulatedConfig field copies verbatim");
    bt::require(!c.grip_pair_force_limit_n.has_value(),
                "grip_pair_force_limit_n stays unset");
    bt::require(c.pedal_attachment == rider::PedalAttachment::flat &&
                        c.saddle_attachment ==
                                rider::SaddleAttachment::flat &&
                        c.grip_attachment == rider::GripAttachment::spring,
                "attachment discriminators copy verbatim");
}

// test_projection_emits_an_explicit_pair_limit — the only optional key:
// absent → unset, a real → decoded.
void test_projection_emits_an_explicit_pair_limit() {
    const auto without = decode_doc(document_with(section_tree()));
    bt::require(!without.rider_contacts->grip_pair_force_limit_n,
                "no limit projects as an absent optional");
    const auto with = decode_doc(document_with(
            section_tree(Pedal::flat, Saddle::flat, Grip::spring, true,
                         300.)));
    bt::require(with.rider_contacts->grip_pair_force_limit_n ==
                        std::optional{300.},
                "an explicit pair limit decodes");
}

// test_project_gates_the_section_on_the_applier — an absent section
// disables the writer; the bare Stepper(path) carries none.
void test_project_gates_the_section_on_the_applier() {
    const bt::TemporaryDirectory dir("rider_gate");
    Value::Object bare;
    bare.emplace_back("schema", setupvalue::integer(2));
    const auto config = decode_doc(Value{std::move(bare)});
    bt::require(!config.rider_contacts.has_value(),
                "an absent rider_contacts key installs no writer");
    auto model = bike::model::compile_mjcf(
            contact_xml(Pedal::flat, Saddle::flat, Grip::spring, false));
    const auto mjb = (dir.path() / "gate.mjb").string();
    mj_saveModel(model.get(), mjb.c_str(), nullptr, 0);
    const Stepper stepper(mjb);
    bt::require(!stepper.has_rider_contacts(),
                "Stepper(path) without config has no rider_contacts");
}

// test_schema_accepts_the_projected_section — decode + Stepper(path,
// config) construction succeeds.
void test_schema_accepts_the_projected_section() {
    const bt::TemporaryDirectory dir("rider_accept");
    const Plant plant = rider_contacts_reference::plant(
            dir, Pedal::flat, Saddle::flat, Grip::spring, false, false, 0.,
            "accepted");
    bt::require(plant.stepper->has_rider_contacts(),
                "the projected section installs the writer");
}

// test_schema_rejects_bad_section_keys[missing|extra|nonstring] — the
// wire's exact-key gate. The 'nonstring' row is a marshal-level contract
// (a JSON object key is always a string): the native equivalent is the
// same unknown-key rejection.
void test_schema_rejects_bad_section_keys(std::string_view mutation) {
    auto section = section_tree();
    if (mutation == "missing") {
        remove_key(section, "arm_reach_m");
        error<std::invalid_argument>(
                [&] { (void)decode_doc(document_with(std::move(section))); },
                "rider_contacts.arm_reach_m");
    } else {
        // 'extra' and 'nonstring' collapse onto the same unknown-key
        // diagnostic; the nonstring row substitutes a numeric key.
        set_key(section, mutation == "extra" ? "misspelled" : "7",
                setupvalue::real(1.));
        const std::string needle = mutation == "extra"
                ? "rider_contacts.misspelled"
                : "rider_contacts";
        error<std::invalid_argument>(
                [&] { (void)decode_doc(document_with(std::move(section))); },
                needle);
    }
}

// test_schema_rejects_bad_section_values[field-bad] — each invalid
// scalar/enum/value row names its public path. The oracle iterates
// validate_config AND Stepper(path, cfg); the native gates split the
// same way — the enum/type rows fail at decode, the domain rows at
// rider::validate inside the writer ctor — so the rejection is proven
// by constructing the Stepper with the decoded document.
void test_schema_rejects_bad_section_values(
        const bt::TemporaryDirectory &dir) {
    auto model = bike::model::compile_mjcf(
            contact_xml(Pedal::flat, Saddle::flat, Grip::spring, false));
    const auto mjb = (dir.path() / "schema_bad.mjb").string();
    mj_saveModel(model.get(), mjb.c_str(), nullptr, 0);
    const auto inf = std::numeric_limits<double>::infinity();
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    const std::pair<std::string_view, Value> rows[] = {
            {"pedal_attachment", setupvalue::text("pin")},
            {"saddle_attachment", setupvalue::text("spindle")},
            {"grip_attachment", setupvalue::text("weld")},
            {"pedal_attachment", setupvalue::integer(7)},
            {"arm_reach_m", setupvalue::real(0.)},
            {"arm_reach_m", setupvalue::real(inf)},
            {"saddle_patch_half_length_m", setupvalue::real(-1e-3)},
            {"support_pad_radius_m", setupvalue::real(0.)},
            {"support_k_n_m", setupvalue::real(-1.)},
            {"support_k_n_m", setupvalue::real(nan)},
            {"support_c_ns_m", setupvalue::real(-1.)},
            {"pedal_c_ns_m", setupvalue::real(-1.)},
            {"support_tangent_k_n_m", setupvalue::real(0.)},
            {"support_mu", setupvalue::real(-1.)},
            {"support_length_m", setupvalue::real(0.)},
            {"grip_k_n_m", setupvalue::real(0.)},
            {"grip_c_ns_m", setupvalue::real(-1.)},
            {"grip_release_distance_m", setupvalue::real(0.)},
            {"grip_capture_distance_m", setupvalue::real(-1.)},
            {"grip_capture_speed_mps", setupvalue::real(-1.)},
            {"grip_pair_force_limit_n", setupvalue::real(-3.)},
            {"grip_pair_force_limit_n", setupvalue::text("unlimited")},
    };
    for (const auto &[field, bad] : rows) {
        auto section = section_tree();
        set_key(section, field, bad);
        error<std::invalid_argument>(
                [&] {
                    Stepper rejected(
                            mjb,
                            decode_doc(document_with(
                                    std::move(section))));
                },
                "rider_contacts." + std::string(field));
    }
}

// test_schema_accepts_every_declared_attachment — every enum label
// decodes.
void test_schema_accepts_every_declared_attachment() {
    for (const char *label : {"flat", "weld", "spindle"}) {
        auto section = section_tree();
        set_key(section, "pedal_attachment", setupvalue::text(label));
        (void)decode_doc(document_with(std::move(section)));
    }
    for (const char *label : {"flat", "weld", "pin"}) {
        auto section = section_tree();
        set_key(section, "saddle_attachment", setupvalue::text(label));
        (void)decode_doc(document_with(std::move(section)));
    }
    for (const char *label : {"spring", "connect"}) {
        auto section = section_tree();
        set_key(section, "grip_attachment", setupvalue::text(label));
        (void)decode_doc(document_with(std::move(section)));
    }
}

// test_pair_limit_is_the_only_optional_key — the optional row deletes
// cleanly; every required row names its path.
void test_pair_limit_is_the_only_optional_key() {
    {
        auto section = section_tree();
        remove_key(section, "grip_pair_force_limit_n");
        (void)decode_doc(document_with(std::move(section)));
    }
    for (const char *key :
         {"arm_reach_m", "saddle_patch_half_length_m",
          "pedal_patch_half_length_m", "support_pad_radius_m",
          "support_k_n_m", "support_c_ns_m", "pedal_c_ns_m",
          "support_tangent_k_n_m", "support_mu", "support_length_m",
          "grip_k_n_m", "grip_c_ns_m", "grip_release_distance_m",
          "grip_capture_distance_m", "grip_capture_speed_mps",
          "pedal_attachment", "saddle_attachment", "grip_attachment"}) {
        auto section = section_tree();
        remove_key(section, key);
        error<std::invalid_argument>(
                [&] { (void)decode_doc(document_with(std::move(section))); },
                "rider_contacts." + std::string(key));
    }
}

// ==========================================================================
// T3b-1 writer core on the tiny plant.
// ==========================================================================

// test_flat_contacts_qfrc_and_diagnostics_bitwise
void test_flat_contacts_qfrc_and_diagnostics_bitwise(
        const bt::TemporaryDirectory &dir) {
    auto golden =
            node(kFile, "test_flat_contacts_qfrc_and_diagnostics_bitwise");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    const auto [fx_qadr, fx] = joint(plant.model.get(), "frame_x");
    const auto [fz_qadr, fz] = joint(plant.model.get(), "frame_z");
    qvel_mut(stepper)[at(fx)] = 0.06;
    qvel_mut(stepper)[at(fz)] = -0.08;
    (void)fx_qadr;
    (void)fz_qadr;
    reset_pair(stepper);
    const auto actual = qfrc(stepper, kDt);
    vector(golden, actual, "qfrc");
    const auto &writer = stepper.rider_contacts();
    diagnostics(golden, writer.state().diagnostics, "root");
    state(golden, writer.state(), writer.probe(), "root");
    golden.done();
}

// test_qfrc_advances_material_state_across_steps — 3 advancing evals;
// pedal_front_tilt tilts between steps 1-2.
void test_qfrc_advances_material_state_across_steps(
        const bt::TemporaryDirectory &dir) {
    auto golden =
            node(kFile, "test_qfrc_advances_material_state_across_steps");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    const auto [tilt, tilt_dof] = joint(plant.model.get(), "pedal_front_tilt");
    const auto [fx_qadr, fx] = joint(plant.model.get(), "frame_x");
    const auto [fz_qadr, fz] = joint(plant.model.get(), "frame_z");
    (void)tilt_dof;
    (void)fx_qadr;
    (void)fz_qadr;
    for (const auto &[step, angle] : {std::pair{0, 0.}, {1, 0.}, {2, 0.7}}) {
        if (step != 0) qpos_mut(stepper)[at(tilt)] = angle;
        qvel_mut(stepper)[at(fx)] = 0.04 * (step + 1);
        qvel_mut(stepper)[at(fz)] = -0.03;
        stepper.data()->time += kDt;
        solved_mirror(stepper);
        const auto actual = qfrc(stepper, kDt);
        vector(golden, actual, "qfrc[" + std::to_string(step) + "]");
        const auto &writer = stepper.rider_contacts();
        diagnostics(golden, writer.state().diagnostics, "root");
        state(golden, writer.state(), writer.probe(), "root");
    }
    golden.done();
}

// test_detailed_false_omits_exactly_the_reference_keys
void test_detailed_false_omits_exactly_the_reference_keys(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_detailed_false_omits_exactly_the_reference_keys");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    const auto actual = qfrc(stepper, kDt, true, false);
    vector(golden, actual, "qfrc");
    const auto &d = stepper.rider_contacts().state().diagnostics;
    diagnostics(golden, d, "root");
    for (const auto &support : d.supports)
        bt::require(!support.detailed && !support.welded,
                    "detailed=False omits the pad detail block");
    for (const auto &side : d.grip_sides)
        bt::require(!side.welded,
                    "detailed=False keeps the spring side layout");
    bt::require(!d.grip.welded,
                "detailed=False keeps the spring aggregate layout");
    golden.done();
}

// test_nonadvancing_probe_publishes_only_probe_fields
void test_nonadvancing_probe_publishes_only_probe_fields(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_nonadvancing_probe_publishes_only_probe_fields");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    const auto probe_qfrc = qfrc(stepper, kDt, false);
    vector(golden, probe_qfrc, "probe qfrc");
    const auto &writer = stepper.rider_contacts();
    bt::require(writer.probe().has_value(),
                "the non-advancing evaluation published a probe");
    diagnostics(golden, writer.probe()->diagnostics, "root");
    enabled_map(golden, writer.probe()->enabled, "root");
    scalar(golden, writer.probe()->delivered_crank_torque_nm,
           "probe_delivered_crank_torque_nm");
    state(golden, writer.state(), writer.probe(), "root");
    // assert_tree(post[key], live[key], f'live.{key}') for every
    // non-probe key — the recorded pre-probe values under 'live.*'.
    state_body(golden, writer.state(), "live");
    golden.done();
}

// test_duplicate_time_guard — a second advancing call at the same
// timestamp is rejected on both owners.
void test_duplicate_time_guard(const bt::TemporaryDirectory &dir) {
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    error<std::invalid_argument>(
            [&] { (void)qfrc(stepper, kDt); },
            "advances only once per timestamp");
}

// test_restart_clock_versus_reset — restart clears only the clock; reset
// re-derives anchors and wipes material state.
void test_restart_clock_versus_reset(const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_restart_clock_versus_reset");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    stepper.mutate([&] { stepper.rider_contacts_restart_clock(); });
    (void)qfrc(stepper, kDt);
    const auto &writer = stepper.rider_contacts();
    state(golden, writer.state(), writer.probe(), "root");
    reset_pair(stepper);
    state(golden, writer.state(), writer.probe(), "root");
    for (const auto &pad : writer.state().supports)
        bt::require(pad.xi == 0.,
                    "reset wipes the pad shear coordinates");
    golden.done();
}

// test_release_all_and_pending_release_loss
void test_release_all_and_pending_release_loss(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_release_all_and_pending_release_loss");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    const auto [fx_qadr, fx] = joint(plant.model.get(), "frame_x");
    (void)fx_qadr;
    qvel_mut(stepper)[at(fx)] = 0.15;
    for (const char *side : {"left", "right"}) {
        const auto [forearm_qpos, dof] =
                body_qpos(plant.model.get(),
                          std::string("rider_forearm_") + side);
        (void)forearm_qpos;
        qvel_mut(stepper)[at(dof + 1)] = 0.1;
    }
    stepper.data()->time += kDt;
    reset_pair(stepper);
    const auto actual = qfrc(stepper, kDt);
    vector(golden, actual, "qfrc");
    stepper.mutate([&] { stepper.rider_contacts_release_all(); });
    const auto &writer = stepper.rider_contacts();
    scalar(golden, writer.state().pending_release_loss_j,
           "pending_release_loss_j");
    for (std::size_t i = 0; i < wc::kContactCount; ++i)
        boolean(golden, writer.state().enabled[i],
                "enabled." + std::string(wc::kContactNames[i]));
    stepper.data()->time += kDt;
    solved_mirror(stepper);
    const auto post = qfrc(stepper, kDt);
    vector(golden, post, "post-release qfrc");
    state(golden, writer.state(), writer.probe(), "root");
    golden.done();
}

// test_set_enabled_capture_success_and_failure
void test_set_enabled_capture_success_and_failure(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_set_enabled_capture_success_and_failure");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    const auto set_enabled = [&](bool enabled) {
        return stepper.mutate([&] {
            return stepper.rider_contacts_set_enabled("grip", enabled);
        });
    };
    bt::require(!set_enabled(false),
                "the grip release attempt reports capture failure");
    bt::require(set_enabled(true),
                "re-enable with hands at the anchors succeeds");
    const auto &writer = stepper.rider_contacts();
    state(golden, writer.state(), writer.probe(), "root");
    bt::require(!set_enabled(false),
                "the second release reports capture failure");
    const auto [qpos, forearm_dof] =
            body_qpos(plant.model.get(), "rider_forearm_left");
    (void)forearm_dof;
    qpos_mut(stepper)[at(qpos)] += 0.2;
    stepper.data()->time += kDt;
    solved_mirror(stepper);
    bt::require(!set_enabled(true),
                "re-enable beyond the capture distance fails");
    state(golden, writer.state(), writer.probe(), "root");
    golden.done();
}

// test_unreachable_hand_releases_and_reports_loss
void test_unreachable_hand_releases_and_reports_loss(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_unreachable_hand_releases_and_reports_loss");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    const auto [qpos, forearm_dof] =
            body_qpos(plant.model.get(), "rider_forearm_right");
    (void)forearm_dof;
    qpos_mut(stepper)[at(qpos + 2)] += 0.4;
    stepper.data()->time += kDt;
    solved_mirror(stepper);
    const auto actual = qfrc(stepper, kDt);
    vector(golden, actual, "release qfrc");
    const auto &writer = stepper.rider_contacts();
    diagnostics(golden, writer.state().diagnostics, "root");
    state(golden, writer.state(), writer.probe(), "root");
    bt::require(!writer.state().enabled[wc::kGripIndex],
                "the unreachable hand releases");
    golden.done();
}

// test_pair_force_limit_releases_both_hands
void test_pair_force_limit_releases_both_hands(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_pair_force_limit_releases_both_hands");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::flat, Saddle::flat, Grip::spring, false, true, 1.);
    Stepper &stepper = *plant.stepper;
    const auto [forearm_qpos, dof] =
            body_qpos(plant.model.get(), "rider_forearm_left");
    (void)forearm_qpos;
    qvel_mut(stepper)[at(dof)] = 0.4;
    reset_pair(stepper);
    const auto actual = qfrc(stepper, kDt);
    vector(golden, actual, "overload qfrc");
    const auto &writer = stepper.rider_contacts();
    diagnostics(golden, writer.state().diagnostics, "root");
    bt::require(!writer.state().enabled[wc::kGripIndex],
                "the pair overload releases the grip");
    bt::require(writer.state().diagnostics.grip_sides[0].overloaded &&
                        writer.state().diagnostics.grip_sides[1]
                                .overloaded,
                "both hands report overloaded");
    golden.done();
}

// test_initialize_settled_state — settled init at t=0, fresh-clock gate
// afterwards.
void test_initialize_settled_state(const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_initialize_settled_state");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    stepper.mutate(
            [&] { stepper.rider_contacts_initialize_settled_state(); });
    const auto &writer = stepper.rider_contacts();
    state(golden, writer.state(), writer.probe(), "root");
    stepper.data()->time += kDt;
    solved_mirror(stepper);
    (void)qfrc(stepper, kDt);
    error<std::invalid_argument>(
            [&] {
                stepper.mutate([&] {
                    stepper.rider_contacts_initialize_settled_state();
                });
            },
            "fresh clock");
    golden.done();
}

// test_linked_attachments_refuse_release — welded/pinned/connected
// contacts cannot be released.
void test_linked_attachments_refuse_release(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_linked_attachments_refuse_release");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::pin, Grip::connect);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    const auto actual = qfrc(stepper, kDt);
    vector(golden, actual, "welded qfrc");
    const auto &writer = stepper.rider_contacts();
    diagnostics(golden, writer.state().diagnostics, "root");
    state(golden, writer.state(), writer.probe(), "root");
    for (const auto name : wc::kContactNames)
        bt::require(stepper.mutate([&] {
                        return stepper.rider_contacts_set_enabled(name,
                                                                  false);
                    }),
                    "linked attachment release is refused but reports "
                    "success: " + std::string(name));
    state(golden, writer.state(), writer.probe(), "root");
    golden.done();
}

// test_welded_branch_null_latch_diagnostics — the pre-settle latch reads
// zeros plus the live translation residual.
void test_welded_branch_null_latch_diagnostics(
        const bt::TemporaryDirectory &dir) {
    // Both attachments share the one pytest node — the golden cursor
    // must survive across iterations.
    auto golden =
            node(kFile, "test_welded_branch_null_latch_diagnostics");
    for (const auto &[pedal, saddle, grip, tag] : {
                 std::tuple{Pedal::weld, Saddle::weld, Grip::connect,
                            "weld_weld"},
                 std::tuple{Pedal::spindle, Saddle::pin, Grip::connect,
                            "spindle_pin"}}) {
        Plant plant = rider_contacts_reference::plant(dir, pedal, saddle,
                                                      grip, false, false,
                                                      0., tag);
        Stepper &stepper = *plant.stepper;
        reset_pair(stepper);
        const auto actual = qfrc(stepper, kDt);
        vector(golden, actual,
               std::string(pedal_name(pedal)) + "/" +
                       saddle_name(saddle) + " qfrc");
        const auto &writer = stepper.rider_contacts();
        diagnostics(golden, writer.state().diagnostics, "root");
        state(golden, writer.state(), writer.probe(), "root");
        const auto &d = writer.state().diagnostics;
        for (const auto &support : d.supports) {
            bt::require(support.normal_load_n == 0. &&
                                !support.would_separate &&
                                support.tangent_n == 0.,
                        "null latch reports zeros for the welded branch");
            bt::require(support.tangent_force_vec ==
                                rider::Vec3{0., 0., 0.},
                        "welded tangent_force_n is the zeros vector");
        }
        bt::require(d.grip.hand_gap_m ==
                            std::max(d.grip_sides[0].hand_gap_m,
                                     d.grip_sides[1].hand_gap_m),
                    "grip.hand_gap_m is the per-side max");
    }
    golden.done();
}

// test_snapshot_round_trip_and_evaluation_determinism
void test_snapshot_round_trip_and_evaluation_determinism(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_snapshot_round_trip_and_evaluation_determinism");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    const auto [fx_qadr, fx] = joint(plant.model.get(), "frame_x");
    (void)fx_qadr;
    qvel_mut(stepper)[at(fx)] = 0.2;
    solved_mirror(stepper);
    (void)qfrc(stepper, kDt);
    const auto &writer = stepper.rider_contacts();
    const Value snapshot =
            rider::encode_rider_contacts_state(writer.state(),
                                               writer.probe());
    state(golden, writer.state(), writer.probe(), "root");
    // A second owner restores the emitted document.
    const auto other = (dir.path() / "contacts_restored.mjb").string();
    mj_saveModel(plant.model.get(), other.c_str(), nullptr, 0);
    Stepper restored(
            other, decode_doc(document_with(
                           section_tree(Pedal::flat, Saddle::flat,
                                        Grip::spring))));
    set_state(restored, snapshot);
    set_state(stepper, snapshot);
    stepper.data()->time += kDt;
    solved_mirror(stepper);
    // _mirror(s, data) on the restored owner: the source stepper's
    // buffers carry bitwise.
    mirror_state(restored, stepper);
    const auto stepper_qfrc = qfrc(stepper, kDt);
    vector(golden, stepper_qfrc, "restored qfrc");
    const auto restored_qfrc = qfrc(restored, kDt);
    vector(golden, restored_qfrc, "oracle-state qfrc");
    const auto &restored_writer = restored.rider_contacts();
    state(golden, restored_writer.state(), restored_writer.probe(),
          "root");
    golden.done();
}

// test_snapshot_owns_its_storage — mutating the emitted document leaves
// the live writer untouched.
void test_snapshot_owns_its_storage(const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_snapshot_owns_its_storage");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    const auto &writer = stepper.rider_contacts();
    Value snapshot = rider::encode_rider_contacts_state(writer.state(),
                                                      writer.probe());
    auto *supports = member_of(snapshot, "supports");
    set_key(*member_of(*supports, "saddle:0"), "xi",
            setupvalue::real(99.));
    auto *grip_xi = member_of(snapshot, "grip_xi_local");
    Value::Array five;
    for (int i = 0; i < 3; ++i) five.push_back(setupvalue::real(5.));
    set_key(*grip_xi, "left", Value{std::move(five)});
    auto *enabled = member_of(snapshot, "enabled");
    set_key(*enabled, "grip", bool_value(false));
    // fresh = rider_contacts_state() again — the encode is a detached
    // copy; the live fields ignore the mutated snapshot.
    const auto &fresh = writer.state();
    bt::require(fresh.supports[0].xi != 99.,
                "the live pad xi ignores snapshot mutation");
    bt::require(fresh.enabled[wc::kGripIndex],
                "the live enabled map ignores snapshot mutation");
    vec3(golden, fresh.grip_xi_local[0], "");
    golden.done();
}

// test_set_state_rejects_invalid_candidates_atomically — every corrupt
// candidate is rejected all-or-nothing.
void test_set_state_rejects_invalid_candidates_atomically(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_set_state_rejects_invalid_candidates_atomically");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    const auto &writer = stepper.rider_contacts();
    const Value baseline = rider::encode_rider_contacts_state(
            writer.state(), writer.probe());
    std::vector<Value> bad;
    {
        auto candidate = baseline;
        set_key(candidate, "enabled",
                object_value({{"saddle", bool_value(true)}}));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(candidate, "supports",
                object_value({{"saddle:0",
                               object_value(
                                       {{"xi", setupvalue::real(0.)},
                                        {"tangent", null_value()}})}}));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        Value::Array zeros;
        for (int i = 0; i < 3; ++i)
            zeros.push_back(setupvalue::real(0.));
        set_key(candidate, "grip_xi_local",
                object_value({{"left", Value{std::move(zeros)}}}));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(candidate, "elastic_energy_j",
                setupvalue::real(
                        std::numeric_limits<double>::quiet_NaN()));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(candidate, "last_time_s", setupvalue::text("now"));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(candidate, "diagnostics", Value{Value::Array{}});
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(candidate, "settled_welds",
                object_value({{"a", setupvalue::integer(1)}}));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(candidate, "probe_enabled",
                object_value({{"grip", bool_value(true)}}));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(candidate, "extra_key", setupvalue::real(0.));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(*member_of(*member_of(candidate, "supports"), "saddle:0"),
                "xi",
                setupvalue::real(
                        std::numeric_limits<double>::infinity()));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        Value::Array pair;
        pair.push_back(setupvalue::real(1.));
        pair.push_back(setupvalue::real(0.));
        set_key(*member_of(*member_of(candidate, "supports"), "saddle:0"),
                "tangent", Value{std::move(pair)});
        bad.push_back(std::move(candidate));
    }
    for (const auto &candidate : bad)
        error<std::invalid_argument>(
                [&] { set_state(stepper, candidate); });
    // Every rejection is all-or-nothing: the live state is untouched.
    state(golden, writer.state(), writer.probe(), "root");
    set_state(stepper, baseline);
    state(golden, writer.state(), writer.probe(), "root");
    golden.done();
}

// test_no_writer_answers_missing_config_errors — every rider_contacts
// face throws std::logic_error naming 'rider_contacts' when the config
// is absent.
void test_no_writer_answers_missing_config_errors(
        const bt::TemporaryDirectory &dir) {
    auto model = bike::model::compile_mjcf(
            contact_xml(Pedal::flat, Saddle::flat, Grip::spring, false));
    const auto mjb = (dir.path() / "bare.mjb").string();
    mj_saveModel(model.get(), mjb.c_str(), nullptr, 0);
    Stepper stepper(mjb);
    for (const auto &call : std::initializer_list<std::function<void()>>{
                 [&] { stepper.rider_contacts_reset(); },
                 [&] { stepper.rider_contacts_restart_clock(); },
                 [&] { stepper.rider_contacts_initialize_settled_state(); },
                 [&] { stepper.rider_contacts_release_all(); },
                 [&] { (void)stepper.rider_contacts_stored_energy(); },
                 [&] { (void)stepper.rider_contacts(); },
                 [&] {
                     (void)stepper.rider_contacts_prepare_attachment_raw();
                 },
                 [&] {
                     (void)stepper.rider_contacts_settle(std::nullopt,
                                                         false, nullptr);
                 },
                 [&] {
                     (void)stepper.rider_contacts_attachment_samples(
                             std::nullopt, false);
                 }})
        error<std::logic_error>(call, "rider_contacts");
    error<std::logic_error>(
            [&] {
                (void)stepper.rider_contacts_set_enabled("grip", false);
            },
            "rider_contacts");
    error<std::logic_error>(
            [&] { (void)qfrc(stepper, 0.0005); }, "rider_contacts");
    error<std::logic_error>(
            [&] {
                (void)rider::decode_rider_contacts_state(
                        Value{}, stepper.rider_contacts());
            },
            "rider_contacts");
}

// ==========================================================================
// T3b-2: the attachment settle/sample face.
// ==========================================================================

// test_prepare_attachment_raw_matches_oracle — 6 attachment combos.
void test_prepare_attachment_raw_matches_oracle(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_prepare_attachment_raw_matches_oracle");
    for (const auto &[pedal, saddle, grip] : {
                 std::tuple{Pedal::weld, Saddle::weld, Grip::connect},
                 std::tuple{Pedal::spindle, Saddle::pin, Grip::connect},
                 std::tuple{Pedal::weld, Saddle::pin, Grip::spring},
                 std::tuple{Pedal::spindle, Saddle::weld, Grip::spring},
                 std::tuple{Pedal::weld, Saddle::flat, Grip::connect},
                 std::tuple{Pedal::flat, Saddle::flat, Grip::spring}}) {
        const std::string tag = std::string("prep_") + pedal_name(pedal) +
                "_" + saddle_name(saddle) + "_" + grip_name(grip);
        Plant plant =
                rider_contacts_reference::plant(dir, pedal, saddle, grip,
                                                false, false, 0., tag);
        Stepper &stepper = *plant.stepper;
        reset_pair(stepper);
        const auto prepared =
                stepper.rider_contacts_prepare_attachment_raw();
        errors(golden, prepared.errors, "errors");
        for (const auto &[name, geometry] : prepared.geometry)
            attachment_geometry(golden, geometry, "root." + name);
    }
    golden.done();
}

// test_settle_welds_latches_solved_reactions — 5 attachment combos.
void test_settle_welds_latches_solved_reactions(
        const bt::TemporaryDirectory &dir) {
    auto golden =
            node(kFile, "test_settle_welds_latches_solved_reactions");
    for (const auto &[pedal, saddle, grip] : {
                 std::tuple{Pedal::weld, Saddle::weld, Grip::connect},
                 std::tuple{Pedal::spindle, Saddle::pin, Grip::connect},
                 std::tuple{Pedal::weld, Saddle::pin, Grip::spring},
                 std::tuple{Pedal::spindle, Saddle::flat, Grip::connect},
                 std::tuple{Pedal::flat, Saddle::flat, Grip::spring}}) {
        const std::string tag = std::string("settle_") +
                pedal_name(pedal) + "_" + saddle_name(saddle) + "_" +
                grip_name(grip);
        Plant plant =
                rider_contacts_reference::plant(dir, pedal, saddle, grip,
                                                false, false, 0., tag);
        Stepper &stepper = *plant.stepper;
        reset_pair(stepper);
        const auto [pelvis, pelvis_dof] =
                body_qpos(plant.model.get(), "rider_pelvis");
        (void)pelvis;
        qfrc_applied_mut(stepper)[at(pelvis_dof + 2)] = -40.;
        solved_mirror(stepper);
        const auto outcome = stepper.mutate([&] {
            return stepper.rider_contacts_settle(std::nullopt, false,
                                                 nullptr);
        });
        settle_result(golden, outcome, "root");
        const auto &writer = stepper.rider_contacts();
        state(golden, writer.state(), writer.probe(), "root");
    }
    golden.done();
}

// test_settled_latch_feeds_next_qfrc — the sensor and welded diagnostics
// read the latch on the next compute_qfrc.
void test_settled_latch_feeds_next_qfrc(const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_settled_latch_feeds_next_qfrc");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::weld, Grip::connect, false, false,
            0., "latch");
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    solved_mirror(stepper);
    const auto outcome = stepper.mutate([&] {
        return stepper.rider_contacts_settle(std::nullopt, false,
                                             nullptr);
    });
    settle_result(golden, outcome, "root");
    const auto actual = qfrc(stepper, kDt);
    vector(golden, actual, "post-settle qfrc");
    const auto &writer = stepper.rider_contacts();
    diagnostics(golden, writer.state().diagnostics, "root");
    state(golden, writer.state(), writer.probe(), "root");
    bt::require(writer.state().settled_welds.has_value(),
                "the settle call latched _settled_welds");
    scalar(golden, writer.state().settled_welds->crank_torque_nm,
           "delivered_crank_torque_nm");
    bt::require(writer.state().delivered_crank_torque_nm ==
                            writer.state().settled_welds->crank_torque_nm &&
                        writer.state().delivered_crank_torque_nm != 0.,
                "delivered_crank_torque_nm is the latched scalar");
    golden.done();
}

// test_attachment_samples_scalar_and_raw_match_oracle — 3 combos x
// {scalar, raw}.
void test_attachment_samples_scalar_and_raw_match_oracle(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_attachment_samples_scalar_and_raw_match_oracle");
    for (const auto &[pedal, saddle, grip] : {
                 std::tuple{Pedal::weld, Saddle::weld, Grip::connect},
                 std::tuple{Pedal::spindle, Saddle::pin, Grip::connect},
                 std::tuple{Pedal::weld, Saddle::pin, Grip::spring}}) {
        const std::string tag = std::string("samples_") +
                pedal_name(pedal) + "_" + saddle_name(saddle) + "_" +
                grip_name(grip);
        Plant plant =
                rider_contacts_reference::plant(dir, pedal, saddle, grip,
                                                false, false, 0., tag);
        Stepper &stepper = *plant.stepper;
        const auto [fz_qadr, fz] = joint(plant.model.get(), "frame_z");
        (void)fz_qadr;
        qvel_mut(stepper)[at(fz)] = -0.05;
        reset_pair(stepper);
        for (const bool raw : {false, true}) {
            const auto measured = stepper.mutate([&] {
                return stepper.rider_contacts_attachment_samples(
                        std::nullopt, raw);
            });
            errors(golden, measured.errors, "errors");
            measurement(golden, measured, "root");
        }
    }
    golden.done();
}

// test_settle_raw_with_prepared_uses_interval_start_geometry
void test_settle_raw_with_prepared_uses_interval_start_geometry(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile,
            "test_settle_raw_with_prepared_uses_interval_start_geometry");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::weld, Grip::connect, false, false,
            0., "prepared_settle");
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    const auto prepared = stepper.rider_contacts_prepare_attachment_raw();
    const auto [pelvis, pelvis_dof] =
            body_qpos(plant.model.get(), "rider_pelvis");
    (void)pelvis_dof;
    qpos_mut(stepper)[at(pelvis + 2)] -= 0.01;
    solved_mirror(stepper);
    const auto outcome = stepper.mutate([&] {
        return stepper.rider_contacts_settle(std::nullopt, true,
                                             &prepared);
    });
    settle_raw_triple(golden, outcome, "root");
    const auto &writer = stepper.rider_contacts();
    state(golden, writer.state(), writer.probe(), "root");
    // The prepared pair may also arrive as the oracle's own dataclass
    // tuple — the same typed struct this side of the binding.
    const auto again = stepper.mutate([&] {
        return stepper.rider_contacts_settle(std::nullopt, true,
                                             &prepared);
    });
    settle_raw_triple(golden, again, "root");
    golden.done();
}

// test_attachment_samples_interval_state_restores_pose
void test_attachment_samples_interval_state_restores_pose(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_attachment_samples_interval_state_restores_pose");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::weld, Grip::connect, false, false,
            0., "interval");
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    solved_mirror(stepper);
    wc::RiderIntervalState interval{
            {stepper.qpos().begin(), stepper.qpos().end()},
            {stepper.qvel().begin(), stepper.qvel().end()}};
    const auto [fx_qadr, fx] = joint(plant.model.get(), "frame_x");
    interval.first[at(fx_qadr)] += 0.02;
    (void)fx;
    const auto measured = stepper.mutate([&] {
        return stepper.rider_contacts_attachment_samples(interval,
                                                         false);
    });
    // The recorded oracle pre-call buffers pin the bitwise restore.
    vector(golden, stepper.qpos(), "oracle qpos");
    vector(golden, stepper.qvel(), "oracle qvel");
    const auto xpos = std::views::counted(
            stepper.data()->xpos, stepper.model()->nbody * 3);
    const std::array<std::size_t, 2> xpos_shape{
            static_cast<std::size_t>(stepper.model()->nbody), 3};
    vector(golden, xpos, xpos_shape, "oracle xpos");
    errors(golden, measured.errors, "errors");
    measurement(golden, measured, "root");
    vector(golden, stepper.qpos(), "qpos restored");
    vector(golden, stepper.qvel(), "qvel restored");
    // Kinematics were refreshed too: the pose-dependent probe repeats.
    const auto probe = qfrc(stepper, kDt, false);
    vector(golden, probe, "kinematics restored");
    golden.done();
}

// test_attachment_samples_interval_restores_through_errors — rejected
// intervals never reach the buffers; collected errors restore the same.
void test_attachment_samples_interval_restores_through_errors(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile,
            "test_attachment_samples_interval_restores_through_errors");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::weld, Grip::connect, true, false,
            0., "interval_err");
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    solved_mirror(stepper);
    const std::size_t nq = stepper.qpos().size();
    const std::size_t nv = stepper.qvel().size();
    for (const auto &bad : {
                 wc::RiderIntervalState{
                         std::vector<double>(nq + 1),
                         std::vector<double>(nv)},
                 wc::RiderIntervalState{std::vector<double>(nq),
                                        std::vector<double>(nv + 1)},
                 wc::RiderIntervalState{std::vector<double>(
                                                nq, std::numeric_limits<
                                                            double>::quiet_NaN()),
                                        std::vector<double>(nv)},
                 wc::RiderIntervalState{
                         std::vector<double>(nq),
                         std::vector<double>(
                                 nv, std::numeric_limits<
                                             double>::infinity())}}) {
        error<std::invalid_argument>([&] {
            stepper.mutate([&] {
                (void)stepper.rider_contacts_attachment_samples(bad,
                                                              false);
            });
        });
    }
    vector(golden, stepper.qpos(), "");
    vector(golden, stepper.qvel(), "");
    wc::RiderIntervalState interval{
            {stepper.qpos().begin(), stepper.qpos().end()},
            {stepper.qvel().begin(), stepper.qvel().end()}};
    const auto [fx_qadr, fx] = joint(plant.model.get(), "frame_x");
    interval.first[at(fx_qadr)] += 0.02;
    (void)fx;
    const auto measured = stepper.mutate([&] {
        return stepper.rider_contacts_attachment_samples(interval,
                                                         false);
    });
    errors(golden, measured.errors, "errors");
    bt::require(measured.errors.size() == 5 &&
                        measured.samples.empty() &&
                        measured.raws.empty(),
                "every measurement collects "
                "':unobservable_attachment_wrench' — never a fabricated "
                "zero");
    bt::require(std::ranges::find(measured.errors,
                                "foot_front:unobservable_"
                                "attachment_wrench") !=
                        measured.errors.end(),
                "the y-slide foot collects the generic spelling");
    measurement(golden, measured, "root");
    vector(golden, stepper.qpos(),
           "qpos restored through collected errors");
    vector(golden, stepper.qvel(),
           "qvel restored through collected errors");
    golden.done();
}

// test_attachment_unobservable_records_errors_not_zeros — the y-slide
// front foot keeps a dof but no observability.
void test_attachment_unobservable_records_errors_not_zeros(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile, "test_attachment_unobservable_records_errors_not_zeros");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::weld, Grip::connect, true, false,
            0., "unobservable");
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    solved_mirror(stepper);
    const auto prepared = stepper.rider_contacts_prepare_attachment_raw();
    errors(golden, prepared.errors, "errors");
    bt::require(prepared.errors.empty(),
                "prepare collects no errors on this plant");
    const rider::AttachmentGeometry *foot_geometry = nullptr;
    for (const auto &[name, geometry] : prepared.geometry) {
        attachment_geometry(golden, geometry, "root." + name);
        if (name == "foot_front") foot_geometry = &geometry;
    }
    bt::require(foot_geometry != nullptr && !foot_geometry->observable,
                "prepare still captures foot_front with "
                "observable=False");
    // Scalar: 'foot_front' becomes an error string, not a zero.
    const auto scalar_measured = stepper.mutate([&] {
        return stepper.rider_contacts_attachment_samples(std::nullopt,
                                                         false);
    });
    errors(golden, scalar_measured.errors, "errors");
    bt::require(std::ranges::find(scalar_measured.errors,
                                "foot_front:unobservable_"
                                "attachment_wrench") !=
                        scalar_measured.errors.end(),
                "the scalar path collects the foot_front error");
    for (const auto &[name, sample] : scalar_measured.samples)
        bt::require(name != "foot_front",
                    "unobservable attachments emit no sample entry");
    measurement(golden, scalar_measured, "root");
    // Raw: the measurement is stored with observable=False.
    const auto raw_measured = stepper.mutate([&] {
        return stepper.rider_contacts_attachment_samples(std::nullopt,
                                                         true);
    });
    errors(golden, raw_measured.errors, "errors");
    bt::require(raw_measured.errors.empty(),
                "the raw path defers validation — no collected errors");
    const rider::AttachmentRaw *foot_raw = nullptr;
    for (const auto &[name, raw] : raw_measured.raws)
        if (name == "foot_front") foot_raw = &raw;
    bt::require(foot_raw != nullptr && !foot_raw->observable,
                "the raw measurement keeps observable=False");
    measurement(golden, raw_measured, "root");
    // settle(raw=True, prepared) takes the same deferred path.
    const auto outcome = stepper.mutate([&] {
        return stepper.rider_contacts_settle(std::nullopt, true,
                                             &prepared);
    });
    settle_raw_triple(golden, outcome, "root");
    bt::require(outcome.measurement.errors.empty(),
                "the settle raw path collects no errors either");
    const auto &writer = stepper.rider_contacts();
    state(golden, writer.state(), writer.probe(), "root");
    golden.done();
}

// test_settled_snapshot_round_trip — latch + published samples survive a
// restore onto a fresh owner bitwise.
void test_settled_snapshot_round_trip(const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_settled_snapshot_round_trip");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::weld, Grip::connect, false, false,
            0., "settled_snap");
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    solved_mirror(stepper);
    (void)stepper.mutate([&] {
        return stepper.rider_contacts_settle(std::nullopt, false,
                                             nullptr);
    });
    (void)qfrc(stepper, kDt);
    const auto &writer = stepper.rider_contacts();
    const Value snapshot = rider::encode_rider_contacts_state(
            writer.state(), writer.probe());
    state(golden, writer.state(), writer.probe(), "root");
    const auto other =
            (dir.path() / "settled_restored.mjb").string();
    mj_saveModel(plant.model.get(), other.c_str(), nullptr, 0);
    const auto config = decode_doc(document_with(
            section_tree(Pedal::weld, Saddle::weld, Grip::connect)));
    Stepper restored(other, config);
    set_state(restored, snapshot);
    // The oracle dict's dataclass-shaped samples parse through the same
    // document decode — one code path this side of the binding.
    Stepper oracle_restored(other, config);
    set_state(oracle_restored, snapshot);
    stepper.data()->time += kDt;
    solved_mirror(stepper);
    mirror_state(restored, stepper);
    mirror_state(oracle_restored, stepper);
    const auto restored_qfrc = qfrc(restored, kDt);
    vector(golden, restored_qfrc, "restored qfrc");
    const auto oracle_qfrc = qfrc(oracle_restored, kDt);
    vector(golden, oracle_qfrc, "oracle-state qfrc");
    const auto &restored_writer = restored.rider_contacts();
    state(golden, restored_writer.state(), restored_writer.probe(),
          "root");
    // The snapshot owns its storage: mutating it leaves the live latch
    // and sample blocks untouched.
    Value mutated = snapshot;
    auto *welds = std::get_if<Value::Array>(
            &member_of(mutated, "settled_welds")->data);
    bt::require(welds != nullptr && welds->size() == 2,
                "settled_welds encodes as the (welds, crank) pair");
    set_key(*member_of(welds->at(0), "saddle"), "normal_n",
            setupvalue::real(-1.));
    set_key(*member_of(*member_of(mutated, "last_attachment_samples"),
                       "foot_front"),
            "normal_n", setupvalue::real(7.));
    Value::Array bogus;
    bogus.push_back(setupvalue::text("bogus"));
    set_key(mutated, "last_attachment_errors",
            Value{std::move(bogus)});
    bt::require(writer.state().settled_welds.has_value(),
                "the latch is still live after snapshot mutation");
    const auto &fresh = writer.state();
    for (const auto &[name, entry] : fresh.settled_welds->entries)
        if (name == "saddle")
            bt::require(entry.normal_n != -1.,
                        "mutating the snapshot leaves the latch "
                        "untouched");
    for (const auto &[name, entry] : fresh.attachment_samples)
        if (name == "foot_front")
            bt::require(std::get<rider::AttachmentSample>(entry)
                                    .normal_n != 7.,
                        "mutating the snapshot leaves the samples "
                        "untouched");
    bt::require(fresh.attachment_errors.empty(),
                "mutating the snapshot leaves the errors empty");
    golden.done();
}

// test_settled_snapshot_rejects_malformed_attachment_fields — malformed
// sample/error/weld fields are rejected; the name-open samples map takes
// unknown-but-valid entries.
void test_settled_snapshot_rejects_malformed_attachment_fields(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(
            kFile,
            "test_settled_snapshot_rejects_malformed_attachment_fields");
    Plant plant = rider_contacts_reference::plant(
            dir, Pedal::weld, Saddle::weld, Grip::connect, false, false,
            0., "settled_bad");
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    solved_mirror(stepper);
    (void)stepper.mutate([&] {
        return stepper.rider_contacts_settle(std::nullopt, false,
                                             nullptr);
    });
    const auto &writer = stepper.rider_contacts();
    const Value baseline = rider::encode_rider_contacts_state(
            writer.state(), writer.probe());
    std::vector<Value> bad;
    {
        auto candidate = baseline;
        set_key(*member_of(candidate, "last_attachment_samples"),
                "foot_front",
                object_value({{"kind", setupvalue::text("foot")}}));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        set_key(*member_of(candidate, "last_attachment_samples"),
                "foot_front", setupvalue::real(5.));
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        Value::Array errors_list;
        errors_list.push_back(setupvalue::text("ok"));
        errors_list.push_back(setupvalue::integer(5));
        set_key(candidate, "last_attachment_errors",
                Value{std::move(errors_list)});
        bad.push_back(std::move(candidate));
    }
    {
        auto candidate = baseline;
        auto *welds = std::get_if<Value::Array>(
                &member_of(candidate, "settled_welds")->data);
        bt::require(welds != nullptr && welds->size() == 2,
                    "settled_welds encodes as the (welds, crank) pair");
        set_key(welds->at(0), "saddle",
                object_value(
                        {{"force_on_rider_n", Value{Value::Array{}}}}));
        bad.push_back(std::move(candidate));
    }
    for (const auto &candidate : bad)
        error<std::invalid_argument>(
                [&] { set_state(stepper, candidate); });
    state(golden, writer.state(), writer.probe(), "root");
    // The samples map is name-open: an unknown-but-valid entry restores.
    {
        auto candidate = baseline;
        const auto *const foot_entry = member_of(
                *member_of(candidate, "last_attachment_samples"),
                "foot_front");
        bt::require(foot_entry != nullptr,
                    "the latch published a foot_front sample");
        auto *const samples = object_of(
                *member_of(candidate, "last_attachment_samples"));
        bt::require(samples != nullptr,
                    "last_attachment_samples is an object");
        samples->emplace_back("extra", *foot_entry);
        set_state(stepper, candidate);
        const bool extra_present = std::ranges::any_of(
                writer.state().attachment_samples,
                [](const auto &entry) { return entry.first == "extra"; });
        bt::require(extra_present,
                    "the name-open map restores the 'extra' entry");
    }
    set_state(stepper, baseline);
    state(golden, writer.state(), writer.probe(), "root");
    golden.done();
}

// test_set_enabled_rejects_invalid_requests — the name/type gate; the
// marshal-only rows (np.bool_, int name) are documented binding
// contracts.
void test_set_enabled_rejects_invalid_requests(
        const bt::TemporaryDirectory &dir) {
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    error<std::invalid_argument>(
            [&] {
                stepper.mutate([&] {
                    (void)stepper.rider_contacts_set_enabled("bogus",
                                                           true);
                });
            },
            "invalid rider contact enable");
}

// test_evaluation_requires_initialized_anchors — no reset: the first
// evaluation throws the anchor-init runtime error.
void test_evaluation_requires_initialized_anchors(
        const bt::TemporaryDirectory &dir) {
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    stepper.note_posvel_stale();
    stepper.forward();
    error<std::runtime_error>(
            [&] { (void)qfrc(stepper, kDt); },
            "initialize rider contact anchors");
}

// test_dt_domain_rejection — the scalar(dt, positive=True) gate.
void test_dt_domain_rejection(const bt::TemporaryDirectory &dir) {
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    reset_pair(stepper);
    for (const double bad :
         {0., -0.001, std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()})
        error<std::invalid_argument>(
                [&] { (void)qfrc(stepper, bad); },
                "invalid rider contact dt");
}

// test_stored_energy_matches_the_ledger — the interface elastic ledger.
void test_stored_energy_matches_the_ledger(
        const bt::TemporaryDirectory &dir) {
    auto golden = node(kFile, "test_stored_energy_matches_the_ledger");
    Plant plant = rider_contacts_reference::plant(dir);
    Stepper &stepper = *plant.stepper;
    const auto [fx_qadr, fx] = joint(plant.model.get(), "frame_x");
    (void)fx_qadr;
    qvel_mut(stepper)[at(fx)] = 0.1;
    stepper.data()->time += kDt;
    reset_pair(stepper);
    (void)qfrc(stepper, kDt);
    // rider_contacts_stored_energy is the binding's one non-mutate face.
    scalar(golden, stepper.rider_contacts_stored_energy(),
           "stored_energy_j");
    golden.done();
}

// test_missing_model_names_reject_construction — a renamed support geom
// fails writer resolution at construction.
void test_missing_model_names_reject_construction(
        const bt::TemporaryDirectory &dir) {
    auto model = bike::model::compile_mjcf(
            [&] {
                auto xml = contact_xml(Pedal::flat, Saddle::flat,
                                       Grip::spring, false);
                const auto at = xml.find("name=\"geom_saddle\"");
                bt::require(at != std::string::npos,
                            "the plant XML carries geom_saddle");
                xml.replace(at, 18, "name=\"geom_saddle_off\"");
                return xml;
            }());
    const auto mjb = (dir.path() / "renamed.mjb").string();
    mj_saveModel(model.get(), mjb.c_str(), nullptr, 0);
    const auto config = decode_doc(document_with(
            section_tree(Pedal::flat, Saddle::flat, Grip::spring, false,
                         0., 0.4)));
    error<std::invalid_argument>(
            [&] { Stepper renamed(mjb, config); },
            "model has no 'geom_saddle'");
    (void)dir;
}

} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported,
// nothing escapes main. The Value-tree builders the compiler inlines
// into the sequence push the frame past the 8 KiB gate — the same
// documented carve-out writer_config.hpp's decode tables carry.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        // The env-fixture block — pure ports (see the file banner).
        test_projection_copies_the_appliers_resolved_setup();
        test_projection_emits_an_explicit_pair_limit();
        test_project_gates_the_section_on_the_applier();
        test_schema_accepts_the_projected_section();
        // One scratch root for the whole sequence — the schema
        // rejections reuse its bare .mjb, the T3b blocks reuse its dir.
        const bt::TemporaryDirectory dir("rider_contacts");
        test_schema_rejects_bad_section_keys("missing");
        test_schema_rejects_bad_section_keys("extra");
        test_schema_rejects_bad_section_keys("nonstring");
        test_schema_rejects_bad_section_values(dir);
        test_schema_accepts_every_declared_attachment();
        test_pair_limit_is_the_only_optional_key();

        // T3b-1 writer core.
        test_flat_contacts_qfrc_and_diagnostics_bitwise(dir);
        test_qfrc_advances_material_state_across_steps(dir);
        test_detailed_false_omits_exactly_the_reference_keys(dir);
        test_nonadvancing_probe_publishes_only_probe_fields(dir);
        test_duplicate_time_guard(dir);
        test_restart_clock_versus_reset(dir);
        test_release_all_and_pending_release_loss(dir);
        test_set_enabled_capture_success_and_failure(dir);
        test_unreachable_hand_releases_and_reports_loss(dir);
        test_pair_force_limit_releases_both_hands(dir);
        test_initialize_settled_state(dir);
        test_linked_attachments_refuse_release(dir);
        test_welded_branch_null_latch_diagnostics(dir);
        test_snapshot_round_trip_and_evaluation_determinism(dir);
        test_snapshot_owns_its_storage(dir);
        test_set_state_rejects_invalid_candidates_atomically(dir);
        test_no_writer_answers_missing_config_errors(dir);
        // T3b-2 settle/sample face.
        test_prepare_attachment_raw_matches_oracle(dir);
        test_settle_welds_latches_solved_reactions(dir);
        test_settled_latch_feeds_next_qfrc(dir);
        test_attachment_samples_scalar_and_raw_match_oracle(dir);
        test_settle_raw_with_prepared_uses_interval_start_geometry(dir);
        test_attachment_samples_interval_state_restores_pose(dir);
        test_attachment_samples_interval_restores_through_errors(dir);
        test_attachment_unobservable_records_errors_not_zeros(dir);
        test_settled_snapshot_round_trip(dir);
        test_settled_snapshot_rejects_malformed_attachment_fields(dir);
        test_set_enabled_rejects_invalid_requests(dir);
        test_evaluation_requires_initialized_anchors(dir);
        test_dt_domain_rejection(dir);
        test_stored_energy_matches_the_ledger(dir);
        test_missing_model_names_reject_construction(dir);
    });
}
NATIVE_DIAG_POP
