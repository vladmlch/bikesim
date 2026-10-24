// rider_contacts_reference_support.hpp — shared plumbing for the
// batch-3.14a rider contact reference ports:
//
//   tests/reference/test_native_rider_contacts.py
//   tests/reference/test_native_rider_contact_math.py
//
// Everything here is test harness: golden-node addressing, the entry
// consumers matching the capture-side `_golden.record`/`assert_tree`
// leaf stream the oracle emits, the tiny `_contact_xml` plant the
// source file builds, and the exception-class checks. No kernel
// behavior is duplicated — every comparison replays the production
// rider:: / writers:: implementations captured by the two-platform
// oracle goldens.
#pragma once

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "support/golden.hpp"
#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "support/temporary_directory.hpp"

#include "configuration/model_config.hpp"
#include "engine_call.hpp"
#include "io/value.hpp"
#include "model/builder.hpp"
#include "model_access.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/contact_config.hpp"
#include "rider/contact_math.hpp"
#include "rider/state_codec.hpp"
#include "rider/support_geometry.hpp"
#include "runtime/setup_values.hpp"
#include "stepper.hpp"
#include "writer_config.hpp"
#include "writers/rider_contacts.hpp"

namespace rider_contacts_reference {

namespace wc = writers;
namespace setupvalue = runtime::setupvalue;
using bike::io::Value;

// ---------------------------------------------------------------------
// Golden node addressing + leaf consumers
// ---------------------------------------------------------------------

[[nodiscard]] inline bt::oracle::Node node(std::string_view file,
                                           std::string_view function,
                                           std::string_view parameters = {}) {
    std::string name = "tests/reference/" + std::string(file) + "::" +
                       std::string(function);
    if (!parameters.empty()) name += "[" + std::string(parameters) + "]";
    return bt::oracle::Node(name);
}

// assert_bitwise_equal(scalar) / a float assert_tree leaf — 0-d <f8.
inline void scalar(bt::oracle::Node &golden, double actual,
                   std::string_view label) {
    const std::array value{actual};
    golden.f64(value, std::span<const std::size_t>{}, label);
}

// assert_bitwise_equal(array) / an ndarray assert_tree leaf — <f8 with
// the recorded C-order shape.
inline void vector(bt::oracle::Node &golden, std::span<const double> actual,
                   std::span<const std::size_t> shape,
                   std::string_view label) {
    golden.f64(actual, shape, label);
}

inline void vector(bt::oracle::Node &golden, std::span<const double> actual,
                   std::string_view label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.f64(actual, shape, label);
}

// _golden.record(np.asarray(bool)) — a 0-d |b1 entry.
inline void boolean(bt::oracle::Node &golden, bool actual,
                    std::string_view label) {
    const std::array<std::byte, 1> bytes{
            std::byte{static_cast<unsigned char>(actual ? 1 : 0)}};
    golden.bytes(bytes, "|b1", std::span<const std::size_t>{}, label);
}

// _golden.record(np.zeros(0, np.int8)) — the None/empty marker leaf.
inline void empty_marker(bt::oracle::Node &golden, std::string_view label) {
    const std::array<std::size_t, 1> shape{0};
    golden.bytes(std::span<const std::byte>{}, "|i1", shape, label);
}

// _golden.record(np.frombuffer(str.encode(), np.uint8)) — a str leaf.
inline void text_leaf(bt::oracle::Node &golden, std::string_view actual,
                      std::string_view label) {
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(std::as_bytes(std::span(actual.data(), actual.size())),
                 "|u1", shape, label);
}

// _golden.record(np.asarray(<python int>)) — a 0-d <i8 entry.
inline void i8_scalar(bt::oracle::Node &golden, std::int64_t actual,
                      std::string_view label) {
    std::array<std::byte, 8> bytes{};
    const auto bits = static_cast<std::uint64_t>(actual);
    for (unsigned shift = 0; shift < 64; shift += 8)
        bytes[shift / 8] =
                static_cast<std::byte>((bits >> shift) & 0xffu);
    golden.bytes(bytes, "<i8", std::span<const std::size_t>{}, label);
}

// _golden.record(np.asarray(<int64 vector>)) — a 1-d <i8 entry.
inline void i8_vector(bt::oracle::Node &golden,
                      std::span<const std::int64_t> actual,
                      std::string_view label) {
    std::vector<std::byte> bytes(actual.size() * 8);
    for (std::size_t i = 0; i < actual.size(); ++i) {
        const auto bits = static_cast<std::uint64_t>(actual[i]);
        for (unsigned shift = 0; shift < 64; shift += 8)
            bytes[i * 8 + shift / 8] =
                    static_cast<std::byte>((bits >> shift) & 0xffu);
    }
    const std::array<std::size_t, 1> shape{actual.size()};
    golden.bytes(bytes, "<i8", shape, label);
}

// An ndarray Vec3 leaf (<f8 [3]).
inline void vec3(bt::oracle::Node &golden, const rider::Vec3 &actual,
                 std::string_view label) {
    const std::array<std::size_t, 1> shape{3};
    golden.f64(actual, shape, label);
}

// A list-of-3 leaf — <f8 [] entries at label[0], [1], [2].
inline void vec3_elements(bt::oracle::Node &golden,
                          const rider::Vec3 &actual,
                          const std::string &label) {
    for (std::size_t i = 0; i < 3; ++i)
        scalar(golden, actual[i], label + "[" + std::to_string(i) + "]");
}

// optional<double> — |i1 [0] for None else <f8 [].
inline void optional_scalar(bt::oracle::Node &golden,
                            const std::optional<double> &actual,
                            std::string_view label) {
    if (actual.has_value())
        scalar(golden, *actual, label);
    else
        empty_marker(golden, label);
}

// optional<Vec3> — |i1 [0] for None else <f8 [3].
inline void optional_vec3(bt::oracle::Node &golden,
                          const std::optional<rider::Vec3> &actual,
                          std::string_view label) {
    if (actual.has_value())
        vec3(golden, *actual, label);
    else
        empty_marker(golden, label);
}

// optional<vector<double>> — |i1 [0] for None else <f8 [n].
inline void optional_vector(bt::oracle::Node &golden,
                            const std::optional<std::vector<double>> &actual,
                            std::string_view label) {
    if (actual.has_value())
        vector(golden, *actual, label);
    else
        empty_marker(golden, label);
}

// f64_input — a Python-synthesized input replayed from the capture.
[[nodiscard]] inline std::vector<double>
input_vector(bt::oracle::Node &golden, std::string_view label) {
    return golden.f64_input(label);
}

// ---------------------------------------------------------------------
// The oracle dict walks — mirroring the binding's dict producers (which
// the pytest's assert_tree verified key-for-key against the oracle).
// ---------------------------------------------------------------------

// patch_dict — {in_platform, normal_load_n, gap_m, point_m(list),
// force_on_rider_n(list), radial_energy_j, shear_energy_j}.
inline void patch_diag(bt::oracle::Node &golden,
                       const wc::RiderContactPatchDiag &p,
                       const std::string &path) {
    boolean(golden, p.in_platform, path + ".in_platform");
    scalar(golden, p.normal_load_n, path + ".normal_load_n");
    scalar(golden, p.gap_m, path + ".gap_m");
    vec3_elements(golden, p.point_m, path + ".point_m");
    vec3_elements(golden, p.force_on_rider_n, path + ".force_on_rider_n");
    scalar(golden, p.radial_energy_j, path + ".radial_energy_j");
    scalar(golden, p.shear_energy_j, path + ".shear_energy_j");
}

// support_diag_dict — the pad and welded layouts the oracle emits.
inline void support_diag(bt::oracle::Node &golden,
                         const wc::RiderContactSupportDiag &d,
                         const std::string &path) {
    boolean(golden, d.enabled, path + ".enabled");
    boolean(golden, d.in_platform, path + ".in_platform");
    scalar(golden, d.normal_load_n, path + ".normal_load_n");
    if (d.welded) {
        boolean(golden, d.would_separate, path + ".would_separate");
        boolean(golden, d.would_slip, path + ".would_slip");
        scalar(golden, d.tangent_n, path + ".tangent_n");
    }
    scalar(golden, d.gap_m, path + ".gap_m");
    scalar(golden, d.vertical_force_on_rider_n,
           path + ".vertical_force_on_rider_n");
    if (d.detailed) {
        if (d.welded)
            vec3(golden, d.tangent_force_vec, path + ".tangent_force_n");
        else
            scalar(golden, d.tangent_force_scalar, path + ".tangent_force_n");
        for (std::size_t i = 0; i < d.patches.size(); ++i)
            patch_diag(golden, d.patches[i],
                       path + ".patches[" + std::to_string(i) + "]");
        vec3_elements(golden, d.force_on_rider_n, path + ".force_on_rider_n");
        vec3_elements(golden, d.force_on_bike_n, path + ".force_on_bike_n");
        vec3_elements(golden, d.moment_nm,
                      path + ".moment_about_rider_origin_nm");
        scalar(golden, d.radial_energy_j, path + ".radial_energy_j");
        scalar(golden, d.shear_energy_j, path + ".shear_energy_j");
        scalar(golden, d.relative_power_w, path + ".relative_power_w");
    }
}

// grip_side_diag_dict — the spring layout always writes the full block;
// the connect layout writes it only when extended.
inline void grip_side_diag(bt::oracle::Node &golden,
                           const wc::RiderContactGripSideDiag &d,
                           const std::string &path) {
    boolean(golden, d.enabled, path + ".enabled");
    boolean(golden, d.reachable, path + ".reachable");
    if (!d.welded) {
        boolean(golden, d.overloaded, path + ".overloaded");
        scalar(golden, d.trial_pair_force_n, path + ".trial_pair_force_n");
        optional_scalar(golden, d.pair_force_limit_n,
                        path + ".pair_force_limit_n");
        scalar(golden, d.release_loss_j, path + ".release_loss_j");
        scalar(golden, d.shoulder_distance_m, path + ".shoulder_distance_m");
        scalar(golden, d.arm_reach_m, path + ".arm_reach_m");
    }
    scalar(golden, d.hand_gap_m, path + ".hand_gap_m");
    if (d.welded && d.extended) {
        boolean(golden, d.overloaded, path + ".overloaded");
        scalar(golden, d.trial_pair_force_n, path + ".trial_pair_force_n");
        optional_scalar(golden, d.pair_force_limit_n,
                        path + ".pair_force_limit_n");
        scalar(golden, d.release_loss_j, path + ".release_loss_j");
        scalar(golden, d.shoulder_distance_m, path + ".shoulder_distance_m");
        scalar(golden, d.arm_reach_m, path + ".arm_reach_m");
    }
    if (!d.welded || d.extended) {
        vec3_elements(golden, d.point_m, path + ".point_m");
        vec3_elements(golden, d.force_on_rider_n, path + ".force_on_rider_n");
        vec3_elements(golden, d.force_on_bike_n, path + ".force_on_bike_n");
        scalar(golden, d.elastic_energy_j, path + ".elastic_energy_j");
    }
}

// grip_diag_dict — the aggregate, never carrying point_m.
inline void grip_diag(bt::oracle::Node &golden,
                      const wc::RiderContactGripDiag &d,
                      const std::string &path) {
    boolean(golden, d.enabled, path + ".enabled");
    boolean(golden, d.reachable, path + ".reachable");
    if (!d.welded) {
        boolean(golden, d.overloaded, path + ".overloaded");
        scalar(golden, d.trial_pair_force_n, path + ".trial_pair_force_n");
        optional_scalar(golden, d.pair_force_limit_n,
                        path + ".pair_force_limit_n");
        scalar(golden, d.release_loss_j, path + ".release_loss_j");
        scalar(golden, d.shoulder_distance_m, path + ".shoulder_distance_m");
        scalar(golden, d.arm_reach_m, path + ".arm_reach_m");
    }
    scalar(golden, d.hand_gap_m, path + ".hand_gap_m");
    if (d.welded && d.extended) {
        boolean(golden, d.overloaded, path + ".overloaded");
        scalar(golden, d.trial_pair_force_n, path + ".trial_pair_force_n");
        optional_scalar(golden, d.pair_force_limit_n,
                        path + ".pair_force_limit_n");
        scalar(golden, d.release_loss_j, path + ".release_loss_j");
        scalar(golden, d.shoulder_distance_m, path + ".shoulder_distance_m");
        scalar(golden, d.arm_reach_m, path + ".arm_reach_m");
    }
    if (!d.welded || d.extended) {
        vec3_elements(golden, d.force_on_rider_n, path + ".force_on_rider_n");
        vec3_elements(golden, d.force_on_bike_n, path + ".force_on_bike_n");
        scalar(golden, d.elastic_energy_j, path + ".elastic_energy_j");
    }
}

// diagnostics_dict — {} until the first evaluation commits.
inline void diagnostics(bt::oracle::Node &golden,
                        const wc::RiderContactsDiagnostics &d,
                        const std::string &path) {
    if (!d.evaluated) return;
    for (std::size_t i = 0; i < wc::kSupportCount; ++i)
        support_diag(golden, d.supports[i],
                     path + "." + std::string(wc::kSupportNames[i]));
    for (std::size_t side = 0; side < wc::kSideCount; ++side)
        grip_side_diag(golden, d.grip_sides[side],
                       path + ".grip_" + std::string(wc::kSideNames[side]));
    grip_diag(golden, d.grip, path + ".grip");
}

// enabled_dict — {saddle, front_pedal, rear_pedal, grip} in contact order.
inline void enabled_map(bt::oracle::Node &golden,
                        const std::array<bool, wc::kContactCount> &enabled,
                        const std::string &path) {
    for (std::size_t i = 0; i < wc::kContactCount; ++i)
        boolean(golden, enabled[i],
                path + "." + std::string(wc::kContactNames[i]));
}

// settled_entry_dict — force_on_rider_n(list), then the support fields.
inline void settled_entry(bt::oracle::Node &golden,
                          const wc::RiderSettledEntry &e,
                          const std::string &path) {
    vec3_elements(golden, e.force_on_rider_n, path + ".force_on_rider_n");
    if (e.support_fields) {
        scalar(golden, e.normal_n, path + ".normal_n");
        scalar(golden, e.tangent_n, path + ".tangent_n");
        boolean(golden, e.would_separate, path + ".would_separate");
        boolean(golden, e.would_slip, path + ".would_slip");
    }
}

// settled_obj — (welds dict, crank) or None.
inline void settled_welds(bt::oracle::Node &golden,
                          const std::optional<wc::RiderSettledWelds> &w,
                          const std::string &path) {
    if (!w.has_value()) {
        empty_marker(golden, path);
        return;
    }
    for (const auto &[name, entry] : w->entries)
        settled_entry(golden, entry, path + "[0]." + name);
    scalar(golden, w->crank_torque_nm, path + "[1]");
}

// geometry_dict — the AttachmentGeometry dataclass fields.
inline void attachment_geometry(bt::oracle::Node &golden,
                                const rider::AttachmentGeometry &g,
                                const std::string &path) {
    i8_scalar(golden, g.eq_id, path + ".eq_id");
    const std::array<std::size_t, 2> rider_shape{g.rider_jac.rows,
                                                 g.rider_jac.columns};
    vector(golden, g.rider_jac.values, rider_shape, path + ".rider_jac");
    i8_vector(golden, g.rider_columns, path + ".rider_columns");
    const std::array<std::size_t, 2> bike_shape{g.bike_jac.rows,
                                                g.bike_jac.columns};
    vector(golden, g.bike_jac.values, bike_shape, path + ".bike_jac");
    i8_vector(golden, g.bike_columns, path + ".bike_columns");
    boolean(golden, g.observable, path + ".observable");
    vector(golden, g.normal, path + ".normal");
    text_leaf(golden, g.kind, path + ".kind");
    boolean(golden, g.rotational, path + ".rotational");
    scalar(golden, g.half_patch_m, path + ".half_patch_m");
    optional_vector(golden, g.pull_direction, path + ".pull_direction");
}

// raw_dict — the AttachmentRaw dataclass fields.
inline void attachment_raw(bt::oracle::Node &golden,
                           const rider::AttachmentRaw &r,
                           const std::string &path) {
    const std::array<std::size_t, 2> rider_shape{r.rider_jac.rows,
                                                 r.rider_jac.columns};
    vector(golden, r.rider_jac.values, rider_shape, path + ".rider_jac");
    vector(golden, r.rider_qfrc, path + ".rider_qfrc");
    const std::array<std::size_t, 2> bike_shape{r.bike_jac.rows,
                                                r.bike_jac.columns};
    vector(golden, r.bike_jac.values, bike_shape, path + ".bike_jac");
    vector(golden, r.bike_qfrc, path + ".bike_qfrc");
    boolean(golden, r.observable, path + ".observable");
    vector(golden, r.normal, path + ".normal");
    text_leaf(golden, r.kind, path + ".kind");
    boolean(golden, r.rotational, path + ".rotational");
    scalar(golden, r.half_patch_m, path + ".half_patch_m");
    scalar(golden, r.gap_m, path + ".gap_m");
    optional_vector(golden, r.pull_direction, path + ".pull_direction");
}

// sample_dict — the budget dataclass's exact fields.
inline void attachment_sample(bt::oracle::Node &golden,
                              const rider::AttachmentSample &s,
                              const std::string &path) {
    text_leaf(golden, s.kind, path + ".kind");
    scalar(golden, s.normal_n, path + ".normal_n");
    scalar(golden, s.tangent_n, path + ".tangent_n");
    scalar(golden, s.moment_nm, path + ".moment_nm");
    scalar(golden, s.gap_m, path + ".gap_m");
    scalar(golden, s.pull_n, path + ".pull_n");
    scalar(golden, s.half_patch_m, path + ".half_patch_m");
}

// attachment_entry_dict — budget sample OR raw block per entry.
inline void attachment_entry(bt::oracle::Node &golden,
                             const wc::RiderAttachmentEntry &entry,
                             const std::string &path) {
    std::visit(
            [&](const auto &v) {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, rider::AttachmentRaw>)
                    attachment_raw(golden, v, path);
                else
                    attachment_sample(golden, v, path);
            },
            entry);
}

// measurement_samples_dict — homogeneous per call by m.raw.
inline void measurement(bt::oracle::Node &golden,
                        const wc::RiderAttachmentMeasurement &m,
                        const std::string &path) {
    if (m.raw) {
        for (const auto &[name, raw] : m.raws)
            attachment_raw(golden, raw, path + "." + name);
    } else {
        for (const auto &[name, sample] : m.samples)
            attachment_sample(golden, sample, path + "." + name);
    }
}

// An errors tuple — |u1 leaves under errors[i].
inline void errors(bt::oracle::Node &golden,
                   const std::vector<std::string> &actual,
                   const std::string &path) {
    for (std::size_t i = 0; i < actual.size(); ++i)
        text_leaf(golden, actual[i], path + "[" + std::to_string(i) + "]");
}

// settle_dict — ordered entries plus the conditional crank tail.
inline void settle_result(bt::oracle::Node &golden,
                          const wc::RiderSettleOutcome &outcome,
                          const std::string &path) {
    for (const auto &[name, entry] : outcome.entries)
        settled_entry(golden, entry, path + "." + name);
    if (outcome.crank_torque_nm_appended)
        scalar(golden, outcome.crank_torque_nm, path + ".crank_torque_nm");
}

// The settle_welds(raw=True) triple: (welds dict, raws dict, errors).
inline void settle_raw_triple(bt::oracle::Node &golden,
                              const wc::RiderSettleOutcome &outcome,
                              const std::string &path) {
    for (const auto &[name, entry] : outcome.entries)
        settled_entry(golden, entry, path + "[0]." + name);
    if (outcome.crank_torque_nm_appended)
        scalar(golden, outcome.crank_torque_nm,
               path + "[0].crank_torque_nm");
    for (const auto &[name, raw] : outcome.measurement.raws)
        attachment_raw(golden, raw, path + "[1]." + name);
    errors(golden, outcome.measurement.errors, path + "[2]");
}

// state_dict's non-probe body — every oracle_state() key through
// last_attachment_errors. The probe test's `live.<key>` walk reuses this
// under a 'live' root: assert_tree(post[key], value, f'live.{key}')
// produces exactly these subtrees one level down.
inline void state_body(bt::oracle::Node &golden,
                       const wc::RiderContactsState &s,
                       const std::string &path) {
    enabled_map(golden, s.enabled, path + ".enabled");
    for (std::size_t key = 0; key < wc::kPadStateCount; ++key) {
        const std::string name =
                std::string(wc::kSupportNames[key / wc::kPadsPerSupport]) +
                ":" + std::to_string(key % wc::kPadsPerSupport);
        const auto &pad = s.supports[key];
        scalar(golden, pad.xi, path + ".supports." + name + ".xi");
        optional_vec3(golden, pad.tangent,
                      path + ".supports." + name + ".tangent");
    }
    for (std::size_t side = 0; side < wc::kSideCount; ++side)
        vec3(golden, s.grip_xi_local[side],
             path + ".grip_xi_local." + std::string(wc::kSideNames[side]));
    for (std::size_t side = 0; side < wc::kSideCount; ++side)
        optional_vec3(
                golden, s.grip_anchor_local[side],
                path + ".grip_anchor_local." +
                        std::string(wc::kSideNames[side]));
    scalar(golden, s.elastic_energy_j, path + ".elastic_energy_j");
    scalar(golden, s.loss_step_j, path + ".loss_step_j");
    scalar(golden, s.radial_dissipation_power_w,
           path + ".radial_dissipation_power_w");
    scalar(golden, s.delivered_crank_torque_nm,
           path + ".delivered_crank_torque_nm");
    optional_scalar(golden, s.last_time_s, path + ".last_time_s");
    scalar(golden, s.pending_release_loss_j,
           path + ".pending_release_loss_j");
    diagnostics(golden, s.diagnostics, path + ".diagnostics");
    settled_welds(golden, s.settled_welds, path + ".settled_welds");
    for (const auto &[name, entry] : s.attachment_samples)
        attachment_entry(golden, entry,
                         path + ".last_attachment_samples." + name);
    errors(golden, s.attachment_errors, path + ".last_attachment_errors");
}

// state_dict — the full oracle_state() document: state_body plus the
// probe trio (dicts when probe is published, three |i1 markers else).
inline void state(bt::oracle::Node &golden, const wc::RiderContactsState &s,
                  const std::optional<wc::RiderContactsProbe> &probe,
                  const std::string &path) {
    state_body(golden, s, path);
    if (probe.has_value()) {
        diagnostics(golden, probe->diagnostics,
                    path + ".probe_diagnostics");
        enabled_map(golden, probe->enabled, path + ".probe_enabled");
        scalar(golden, probe->delivered_crank_torque_nm,
               path + ".probe_delivered_crank_torque_nm");
    } else {
        empty_marker(golden, path + ".probe_diagnostics");
        empty_marker(golden, path + ".probe_enabled");
        empty_marker(golden, path + ".probe_delivered_crank_torque_nm");
    }
}

// ---------------------------------------------------------------------
// The tiny _contact_xml plant + Stepper construction
// ---------------------------------------------------------------------

enum class Pedal { flat, weld, spindle };
enum class Saddle { flat, weld, pin };
enum class Grip { spring, connect };

[[nodiscard]] inline const char *pedal_name(Pedal p) {
    switch (p) {
    case Pedal::flat: return "flat";
    case Pedal::weld: return "weld";
    case Pedal::spindle: return "spindle";
    }
    return "?";
}
[[nodiscard]] inline const char *saddle_name(Saddle s) {
    switch (s) {
    case Saddle::flat: return "flat";
    case Saddle::weld: return "weld";
    case Saddle::pin: return "pin";
    }
    return "?";
}
[[nodiscard]] inline const char *grip_name(Grip g) {
    switch (g) {
    case Grip::spring: return "spring";
    case Grip::connect: return "connect";
    }
    return "?";
}

// _contact_xml — the literal string the pytest file assembles, byte-for-
// byte in structure (equalities appended per attachment kind).
[[nodiscard]] inline std::string contact_xml(Pedal pedal, Saddle saddle,
                                             Grip grip, bool front_slide) {
    const std::string front_joint =
            front_slide ? R"(<joint name="foot_front_y" type="slide" axis="0 1 0"/>)"
                        : "<freejoint/>";
    std::string equalities;
    if (pedal == Pedal::weld)
        for (const char *s : {"front", "rear"})
            equalities += "<weld name=\"weld_foot_" + std::string(s) +
                          "\" body1=\"rider_foot_" + s +
                          "\" body2=\"pedal_" + s + "\"/>";
    else if (pedal == Pedal::spindle)
        for (const auto &[s, anchor] : {std::pair{"front", "0.25 0.04 0.795"},
                                        std::pair{"rear", "0.25 -0.04 0.51"}})
            equalities += "<connect name=\"connect_foot_" + std::string(s) +
                          "\" body1=\"rider_foot_" + s +
                          "\" body2=\"pedal_" + s + "\" anchor=\"" + anchor +
                          "\"/>";
    if (saddle == Saddle::weld)
        equalities +=
                R"(<weld name="weld_saddle" body1="rider_pelvis" body2="frame"/>)";
    else if (saddle == Saddle::pin)
        equalities +=
                R"(<connect name="connect_saddle" body1="rider_pelvis" body2="frame" anchor="0 0 1.47"/>)";
    if (grip == Grip::connect)
        for (const auto &[s, y] : {std::pair{"left", "0.12"},
                                   std::pair{"right", "-0.12"}})
            equalities += "<connect name=\"connect_grip_" + std::string(s) +
                          "\" body1=\"rider_forearm_" + s +
                          "\" body2=\"steer\" anchor=\"0.44 " + y +
                          " 1.5\"/>";
    const std::string eq =
            equalities.empty() ? "" : "<equality>" + equalities + "</equality>";
    return std::string(R"xml(<mujoco>
  <option gravity="0 0 -9.81" timestep="0.0005"/>
  <worldbody>
    <body name="frame" pos="0 0 1">
      <joint name="frame_x" type="slide" axis="1 0 0"/>
      <joint name="frame_z" type="slide" axis="0 0 1"/>
      <joint name="frame_lean" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
      <geom name="geom_saddle" type="box" size="0.15 0.06 0.03" pos="0 0 0.45"
            contype="0" conaffinity="0"/>
      <body name="steer" pos="0.45 0 0.5">
        <joint name="steer_hinge" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.05 0.05 0.05"/>
        <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
      </body>
      <body name="crank" pos="0.25 0 -0.35">
        <joint name="crank_spin" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="1" diaginertia="0.05 0.05 0.05"/>
        <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
        <body name="pedal_front" pos="0 0.04 0.14">
          <joint name="pedal_front_tilt" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="0.2" diaginertia="0.005 0.005 0.005"/>
          <geom name="geom_pedal_front" type="box" size="0.09 0.05 0.012"
                contype="0" conaffinity="0"/>
        </body>
        <body name="pedal_rear" pos="0 -0.04 -0.14">
          <joint name="pedal_rear_tilt" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 0" mass="0.2" diaginertia="0.005 0.005 0.005"/>
          <geom name="geom_pedal_rear" type="box" size="0.09 0.05 0.012"
                contype="0" conaffinity="0"/>
        </body>
      </body>
    </body>
    <body name="rider_pelvis" pos="0 0 1.51">
      <freejoint/>
      <inertial pos="0 0 0" mass="20" diaginertia="1 1 1"/>
      <site name="site_rider_saddle" pos="0 0 -0.04"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_foot_front" pos="0.25 0.04 0.83">
      )xml") +
           front_joint + R"xml(
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_sole_front" pos="0 0 -0.035"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_foot_rear" pos="0.25 -0.04 0.545">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_sole_rear" pos="0 0 -0.035"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_upper_arm_left" pos="0.32 0.12 1.52">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.5" diaginertia="0.02 0.02 0.02"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_upper_arm_right" pos="0.32 -0.12 1.52">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.5" diaginertia="0.02 0.02 0.02"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_forearm_left" pos="0.44 0.12 1.5">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_grip_left"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
    <body name="rider_forearm_right" pos="0.44 -0.12 1.5">
      <freejoint/>
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <site name="site_rider_grip_right"/>
      <geom type="sphere" size="0.02" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  )xml" +
           eq + "\n</mujoco>";
}

// project_rider_contacts(applier) for the _pose() defaults — the wire
// dict the pytest file decodes into Stepper(path, {'schema': 2,
// 'rider_contacts': section}). arm_reach_m is the resolved seated pose
// reach (0.447213595499958), the literals ArticulatedConfig() carries.
[[nodiscard]] inline Value contacts_section(Pedal pedal, Saddle saddle,
                                            Grip grip,
                                            bool pair_limit = false,
                                            double pair_limit_n = 0.,
                                            double arm_reach =
                                                    0.447213595499958) {
    Value::Object section;
    section.emplace_back("arm_reach_m", setupvalue::real(arm_reach));
    section.emplace_back("saddle_patch_half_length_m",
                         setupvalue::real(0.045));
    section.emplace_back("pedal_patch_half_length_m",
                         setupvalue::real(0.025));
    section.emplace_back("support_pad_radius_m", setupvalue::real(0.02));
    section.emplace_back("support_k_n_m", setupvalue::real(100000.0));
    section.emplace_back("support_c_ns_m", setupvalue::real(500.0));
    section.emplace_back("pedal_c_ns_m", setupvalue::real(100.0));
    section.emplace_back("support_tangent_k_n_m",
                         setupvalue::real(20000.0));
    section.emplace_back("support_mu", setupvalue::real(0.8));
    section.emplace_back("support_length_m", setupvalue::real(0.1));
    section.emplace_back("grip_k_n_m", setupvalue::real(4000.0));
    section.emplace_back("grip_c_ns_m", setupvalue::real(150.0));
    section.emplace_back("grip_release_distance_m",
                         setupvalue::real(0.12));
    // The projection emits the key unconditionally — JSON null when the
    // config carries no limit, the finite real otherwise.
    section.emplace_back("grip_pair_force_limit_n",
                         pair_limit ? setupvalue::real(pair_limit_n)
                                    : Value{});
    section.emplace_back("grip_capture_distance_m",
                         setupvalue::real(0.02));
    section.emplace_back("grip_capture_speed_mps",
                         setupvalue::real(0.2));
    section.emplace_back("pedal_attachment",
                         setupvalue::text(pedal_name(pedal)));
    section.emplace_back("saddle_attachment",
                         setupvalue::text(saddle_name(saddle)));
    section.emplace_back("grip_attachment",
                         setupvalue::text(grip_name(grip)));
    return Value{std::move(section)};
}

// The {'schema': 2, 'rider_contacts': section} document.
[[nodiscard]] inline Value config_doc(Pedal pedal, Saddle saddle, Grip grip,
                                      bool pair_limit = false,
                                      double pair_limit_n = 0.,
                                      double arm_reach =
                                              0.447213595499958) {
    Value::Object root;
    root.emplace_back("schema", setupvalue::integer(2));
    root.emplace_back("rider_contacts",
                      contacts_section(pedal, saddle, grip, pair_limit,
                                       pair_limit_n, arm_reach));
    return Value{std::move(root)};
}

// The tiny plant: compiled MJCF, saved .mjb, decoded wire config, live
// Stepper — the C++ twin of _plant(). Construction order matters:
// data outlives the stepper, the stepper outlives both pointers' uses.
struct Plant {
    bike::model::ModelOwner model;
    std::unique_ptr<mjData, bt::DataDeleter> data;
    std::string mjb;
    std::unique_ptr<Stepper> stepper;
};

[[nodiscard]] inline Plant plant(const bt::TemporaryDirectory &dir,
                                 Pedal pedal = Pedal::flat,
                                 Saddle saddle = Saddle::flat,
                                 Grip grip = Grip::spring,
                                 bool front_slide = false,
                                 bool pair_limit = false,
                                 double pair_limit_n = 0.,
                                 std::string_view name = "contacts",
                                 double arm_reach =
                                         0.447213595499958) {
    Plant out;
    out.model = bike::model::compile_mjcf(
            contact_xml(pedal, saddle, grip, front_slide));
    out.data.reset(engine::make_data(out.model.get()));
    mj_forward(out.model.get(), out.data.get());
    out.mjb = (dir.path() / (std::string(name) + ".mjb")).string();
    mj_saveModel(out.model.get(), out.mjb.c_str(), nullptr, 0);
    out.stepper = std::make_unique<Stepper>(
            out.mjb,
            nativecfg::decode(
                    config_doc(pedal, saddle, grip, pair_limit,
                               pair_limit_n, arm_reach)));
    return out;
}

// _joint — (qposadr, dofadr) for a named scalar joint. mjModel arrays
// are read through model_access's span gate (-Wunsafe-buffer-usage).
[[nodiscard]] inline std::pair<int, int> joint(const mjModel *model,
                                               std::string_view name) {
    const int id = mj_name2id(model, mjOBJ_JOINT,
                              std::string(name).c_str());
    model_access::require_id(id, model->njnt, name);
    const auto at = static_cast<std::size_t>(id);
    return {static_cast<int>(model_access::readonly_buffer(
                    model->jnt_qposadr, model->njnt)[at]),
            static_cast<int>(model_access::readonly_buffer(
                    model->jnt_dofadr, model->njnt)[at])};
}

// _body_qpos — the freejoint's (qposadr, dofadr) pair for a named body.
[[nodiscard]] inline std::pair<int, int> body_qpos(const mjModel *model,
                                                  std::string_view name) {
    const int body = mj_name2id(model, mjOBJ_BODY,
                                std::string(name).c_str());
    model_access::require_id(body, model->nbody, name);
    const int jid = model_access::readonly_buffer(
            model->body_jntadr, model->nbody)[
            static_cast<std::size_t>(body)];
    bt::require(model_access::readonly_buffer(model->jnt_type,
                                              model->njnt)[
                        static_cast<std::size_t>(jid)] == mjJNT_FREE,
                (std::string("expected a freejoint body: ") +=
                 std::string(name)));
    return {static_cast<int>(model_access::readonly_buffer(
                    model->jnt_qposadr, model->njnt)[
                    static_cast<std::size_t>(jid)]),
            static_cast<int>(model_access::readonly_buffer(
                    model->jnt_dofadr, model->njnt)[
                    static_cast<std::size_t>(jid)])};
}

// int → span index (the oracle's bare qpos[i] subscripts): the span
// gate wants an unsigned element index.
[[nodiscard]] constexpr std::size_t at(int index) noexcept {
    return static_cast<std::size_t>(index);
}

// mjData write faces through the same span gate — the oracle's bare
// data.qpos[data.qvel/...] index assignments.
[[nodiscard]] inline std::span<double> qpos_mut(Stepper &stepper) {
    return model_access::mutable_buffer(stepper.data()->qpos,
                                        stepper.model()->nq, "qpos");
}

[[nodiscard]] inline std::span<double> qvel_mut(Stepper &stepper) {
    return model_access::mutable_buffer(stepper.data()->qvel,
                                        stepper.model()->nv, "qvel");
}

[[nodiscard]] inline std::span<double> qfrc_applied_mut(Stepper &stepper) {
    return model_access::mutable_buffer(stepper.data()->qfrc_applied,
                                        stepper.model()->nv,
                                        "qfrc_applied");
}

// _reset_pair: forward on the current buffers, then the writer reset
// through the binding's mutate boundary.
inline void reset_pair(Stepper &stepper) {
    stepper.note_posvel_stale();
    stepper.forward();
    stepper.mutate([&] { stepper.rider_contacts_reset(); });
}

// _solved_mirror for the single-owner C++ port: the stepper's own pose/
// velocity/inputs are forwarded so the efc arena is the one settle reads.
inline void solved_mirror(Stepper &stepper) {
    stepper.note_posvel_stale();
    stepper.forward();
}

// data.qpos[adr] write + forward — the oracle's `data.qpos[...]=x;
// mujoco.mj_forward` sequence on the single owner.
inline void set_qpos_forward(Stepper &stepper, int adr, double value) {
    qpos_mut(stepper)[static_cast<std::size_t>(adr)] = value;
    stepper.note_posvel_stale();
    stepper.forward();
}

// _mirror(stepper, data) on a SECOND Stepper owner: set_state +
// forward carries the source owner's buffers bitwise — the same move
// the pytest's mirror performs from the oracle's mjData.
inline void mirror_state(Stepper &dst, const Stepper &src) {
    dst.set_state(
            src.qpos(), src.qvel(),
            std::views::counted(src.data()->act, src.model()->na),
            std::views::counted(src.data()->qacc_warmstart,
                                src.model()->nv),
            src.data()->time);
    dst.forward();
}

// Exception-class checks — ValueError arrives as std::invalid_argument;
// RuntimeError as std::runtime_error; the missing-writer gate as
// std::logic_error. The needle pins the oracle's `match=` text.
template <class Exception, class F>
void error(F &&action, std::string_view needle = {},
           const std::source_location &loc =
                   std::source_location::current()) {
    bool thrown = false;
    try {
        std::forward<F>(action)();
    } catch (const Exception &failure) {
        thrown = true;
        bt::require(needle.empty() ||
                            std::string_view(failure.what())
                                            .find(needle) !=
                                    std::string_view::npos,
                    (std::string("exception text lacks '") +=
                     std::string(needle)) += "'");
    }
    bt::require(thrown, (std::string("expected rejection at ") +=
                         loc.file_name()) += ":" +
                                std::to_string(loc.line()));
}

// Bitwise comparison of two spans — the port's self-consistency proofs
// (probe isolation, restore equality) the pytest does oracle-internally.
[[nodiscard]] inline bool bitwise_equal(std::span<const double> a,
                                        std::span<const double> b) {
    return a.size() == b.size() &&
           std::ranges::equal(std::as_bytes(a), std::as_bytes(b));
}

} // namespace rider_contacts_reference
