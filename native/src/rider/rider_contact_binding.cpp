// rider_contact_binding.cpp — the T3b-1 FFI seam for the model-owned
// RiderContactWriter. Three concerns live here and nowhere else:
//
//   * Method bindings on the Stepper class (qfrc, lifecycle, enable,
//     diagnostics, state round-trip) — each delegates to Stepper's writer
//     accessor, so a missing 'rider_contacts' config raises the shared
//     logic_error naming the section.
//   * Diagnostics/state serialization: the Python-visible trees are built
//     fresh on every call — nothing returns a view into writer storage, so
//     a caller mutating the snapshot cannot corrupt the writer.
//   * State candidate parsing: every field is read, type-checked and
//     domain-checked into a staging RiderContactsState BEFORE the live
//     writer is touched — a rejected candidate leaves the writer exactly
//     as it was (the transaction boundary the oracle's attribute model
//     gets for free).
//
// Wire shapes mirror the oracle's attributes 1:1 — dict key sets, ndarray
// vs list vs scalar, and the tuple/None sentinels are all contract.
#include "rider_contact_binding.hpp"

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"
#include "../diag.hpp"
#include "../stepper.hpp"
#include "../writers/rider_contacts.hpp"

#include <algorithm>
#include <array>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace {
    using writers::RiderContactWriter;
    namespace wc = writers;

    // ---- emission -------------------------------------------------------

    // Owning 1-D float64 ndarray (np.asarray / np.zeros(3) parity).
    [[nodiscard]] nb::ndarray<nb::numpy, double, nb::shape<-1>>
    owned_vec(const rider::Vec3 &v) {
        return wire::owned_array<double>(
            std::span<const double>{v.data(), v.size()});
    }

    // .tolist() parity — a Python list of Python floats.
    [[nodiscard]] nb::list vec_list(const rider::Vec3 &v) {
        nb::list out;
        for (const double x : v) out.append(x);
        return out;
    }

    [[nodiscard]] nb::object optional_vec(const std::optional<rider::Vec3> &v) {
        if (!v.has_value()) return nb::none();
        return owned_vec(*v).cast();
    }

    [[nodiscard]] nb::object optional_number(std::optional<double> v) {
        if (!v.has_value()) return nb::none();
        return nb::float_(*v);
    }

    [[nodiscard]] nb::dict enabled_dict(const std::array<bool, wc::kContactCount> &e) {
        nb::dict out;
        for (std::size_t i = 0; i < wc::kContactCount; ++i)
            out[wc::kContactNames[i].data()] = e[i];
        return out;
    }

    [[nodiscard]] nb::dict patch_dict(const wc::RiderContactPatchDiag &p) {
        nb::dict out;
        out["in_platform"] = p.in_platform;
        out["normal_load_n"] = p.normal_load_n;
        out["gap_m"] = p.gap_m;
        out["point_m"] = vec_list(p.point_m);
        out["force_on_rider_n"] = vec_list(p.force_on_rider_n);
        out["radial_energy_j"] = p.radial_energy_j;
        out["shear_energy_j"] = p.shear_energy_j;
        return out;
    }

    [[nodiscard]] nb::dict support_diag_dict(const wc::RiderContactSupportDiag &d) {
        nb::dict out;
        out["enabled"] = d.enabled;
        out["in_platform"] = d.in_platform;
        out["normal_load_n"] = d.normal_load_n;
        if (d.welded) {
            out["would_separate"] = d.would_separate;
            out["would_slip"] = d.would_slip;
            out["tangent_n"] = d.tangent_n;
        }
        out["gap_m"] = d.gap_m;
        out["vertical_force_on_rider_n"] = d.vertical_force_on_rider_n;
        if (d.detailed) {
            // np.zeros(3) on the welded layout, the scalar on the pad one.
            if (d.welded)
                out["tangent_force_n"] = owned_vec(d.tangent_force_vec);
            else
                out["tangent_force_n"] = d.tangent_force_scalar;
            nb::list patches;
            for (const auto &p : d.patches) patches.append(patch_dict(p));
            out["patches"] = std::move(patches);
            out["force_on_rider_n"] = vec_list(d.force_on_rider_n);
            out["force_on_bike_n"] = vec_list(d.force_on_bike_n);
            out["moment_about_rider_origin_nm"] = vec_list(d.moment_nm);
            out["radial_energy_j"] = d.radial_energy_j;
            out["shear_energy_j"] = d.shear_energy_j;
            out["relative_power_w"] = d.relative_power_w;
        }
        return out;
    }

    [[nodiscard]] nb::dict
    grip_side_diag_dict(const wc::RiderContactGripSideDiag &d) {
        nb::dict out;
        out["enabled"] = d.enabled;
        out["reachable"] = d.reachable;
        if (!d.welded) {
            // The spring side dict always carries the full block — the
            // connect layout writes it only for a detailed evaluation.
            out["overloaded"] = d.overloaded;
            out["trial_pair_force_n"] = d.trial_pair_force_n;
            out["pair_force_limit_n"] = optional_number(d.pair_force_limit_n);
            out["release_loss_j"] = d.release_loss_j;
            out["shoulder_distance_m"] = d.shoulder_distance_m;
            out["arm_reach_m"] = d.arm_reach_m;
        }
        out["hand_gap_m"] = d.hand_gap_m;
        if (d.welded && d.extended) {
            out["overloaded"] = d.overloaded;
            out["trial_pair_force_n"] = d.trial_pair_force_n;
            out["pair_force_limit_n"] = optional_number(d.pair_force_limit_n);
            out["release_loss_j"] = d.release_loss_j;
            out["shoulder_distance_m"] = d.shoulder_distance_m;
            out["arm_reach_m"] = d.arm_reach_m;
        }
        if (!d.welded || d.extended) {
            out["point_m"] = vec_list(d.point_m);
            out["force_on_rider_n"] = vec_list(d.force_on_rider_n);
            out["force_on_bike_n"] = vec_list(d.force_on_bike_n);
            out["elastic_energy_j"] = d.elastic_energy_j;
        }
        return out;
    }

    [[nodiscard]] nb::dict grip_diag_dict(const wc::RiderContactGripDiag &d) {
        nb::dict out;
        out["enabled"] = d.enabled;
        out["reachable"] = d.reachable;
        if (!d.welded) {
            out["overloaded"] = d.overloaded;
            out["trial_pair_force_n"] = d.trial_pair_force_n;
            out["pair_force_limit_n"] = optional_number(d.pair_force_limit_n);
            out["release_loss_j"] = d.release_loss_j;
            out["shoulder_distance_m"] = d.shoulder_distance_m;
            out["arm_reach_m"] = d.arm_reach_m;
        }
        out["hand_gap_m"] = d.hand_gap_m;
        if (d.welded && d.extended) {
            out["overloaded"] = d.overloaded;
            out["trial_pair_force_n"] = d.trial_pair_force_n;
            out["pair_force_limit_n"] = optional_number(d.pair_force_limit_n);
            out["release_loss_j"] = d.release_loss_j;
            out["shoulder_distance_m"] = d.shoulder_distance_m;
            out["arm_reach_m"] = d.arm_reach_m;
        }
        if (!d.welded || d.extended) {
            out["force_on_rider_n"] = vec_list(d.force_on_rider_n);
            out["force_on_bike_n"] = vec_list(d.force_on_bike_n);
            out["elastic_energy_j"] = d.elastic_energy_j;
        }
        return out;
    }

    [[nodiscard]] nb::dict
    diagnostics_dict(const wc::RiderContactsDiagnostics &d) {
        nb::dict out;
        if (!d.evaluated) return out;
        for (std::size_t i = 0; i < wc::kSupportCount; ++i)
            out[wc::kSupportNames[i].data()] =
                support_diag_dict(d.supports[i]);
        for (std::size_t side = 0; side < wc::kSideCount; ++side)
            out[(std::string("grip_") +
                 std::string(wc::kSideNames[side])).c_str()] =
                grip_side_diag_dict(d.grip_sides[side]);
        out["grip"] = grip_diag_dict(d.grip);
        return out;
    }

    [[nodiscard]] nb::dict settled_entry_dict(const wc::RiderSettledEntry &e) {
        nb::dict out;
        out["force_on_rider_n"] = vec_list(e.force_on_rider_n);
        if (e.support_fields) {
            out["normal_n"] = e.normal_n;
            out["tangent_n"] = e.tangent_n;
            out["would_separate"] = e.would_separate;
            out["would_slip"] = e.would_slip;
        }
        return out;
    }

    [[nodiscard]] nb::object
    settled_obj(const std::optional<wc::RiderSettledWelds> &w) {
        if (!w.has_value()) return nb::none();
        nb::dict welds;
        for (const auto &[name, entry] : w->entries)
            welds[name.c_str()] = settled_entry_dict(entry);
        return nb::make_tuple(std::move(welds), w->crank_torque_nm);
    }

    [[nodiscard]] nb::dict sample_dict(const rider::AttachmentSample &s) {
        nb::dict out;
        out["kind"] = s.kind;
        out["normal_n"] = s.normal_n;
        out["tangent_n"] = s.tangent_n;
        out["moment_nm"] = s.moment_nm;
        out["gap_m"] = s.gap_m;
        out["pull_n"] = s.pull_n;
        out["half_patch_m"] = s.half_patch_m;
        return out;
    }

    // The oracle-shaped snapshot: same key set, same ndarray/list/scalar
    // split as oracle_state() in the reference test derives.
    [[nodiscard]] nb::dict
    state_dict(const wc::RiderContactsState &s,
               const std::optional<wc::RiderContactsProbe> &probe) {
        nb::dict out;
        out["enabled"] = enabled_dict(s.enabled);
        nb::dict supports;
        for (std::size_t si = 0; si < wc::kSupportCount; ++si) {
            for (std::size_t pad = 0; pad < wc::kPadsPerSupport; ++pad) {
                const auto &state =
                    s.supports[wc::kPadsPerSupport * si + pad];
                nb::dict entry;
                entry["xi"] = state.xi;
                entry["tangent"] = optional_vec(state.tangent);
                supports[(std::string(wc::kSupportNames[si]) + ":" +
                          std::to_string(pad)).c_str()] = std::move(entry);
            }
        }
        out["supports"] = std::move(supports);
        nb::dict xi;
        for (std::size_t side = 0; side < wc::kSideCount; ++side)
            xi[wc::kSideNames[side].data()] = owned_vec(s.grip_xi_local[side]);
        out["grip_xi_local"] = std::move(xi);
        nb::dict anchors;
        for (std::size_t side = 0; side < wc::kSideCount; ++side)
            anchors[wc::kSideNames[side].data()] =
                optional_vec(s.grip_anchor_local[side]);
        out["grip_anchor_local"] = std::move(anchors);
        out["elastic_energy_j"] = s.elastic_energy_j;
        out["loss_step_j"] = s.loss_step_j;
        out["radial_dissipation_power_w"] = s.radial_dissipation_power_w;
        out["delivered_crank_torque_nm"] = s.delivered_crank_torque_nm;
        out["last_time_s"] = optional_number(s.last_time_s);
        out["pending_release_loss_j"] = s.pending_release_loss_j;
        out["diagnostics"] = diagnostics_dict(s.diagnostics);
        out["settled_welds"] = settled_obj(s.settled_welds);
        nb::dict samples;
        for (const auto &[name, sample] : s.attachment_samples)
            samples[name.c_str()] = sample_dict(sample);
        out["last_attachment_samples"] = std::move(samples);
        nb::list errors;
        for (const std::string &e : s.attachment_errors)
            errors.append(e.c_str());
        // tuple(errors) — the oracle's last_attachment_errors is a tuple,
        // and the tree comparator is type-strict.
        out["last_attachment_errors"] =
            nb::steal<nb::tuple>(PySequence_Tuple(errors.ptr()));
        out["probe_diagnostics"] =
            probe.has_value() ? nb::object(diagnostics_dict(probe->diagnostics))
                              : nb::object(nb::none());
        out["probe_enabled"] = probe.has_value()
                                   ? nb::object(enabled_dict(probe->enabled))
                                   : nb::object(nb::none());
        out["probe_delivered_crank_torque_nm"] =
            probe.has_value()
                ? nb::object(nb::float_(probe->delivered_crank_torque_nm))
                : nb::object(nb::none());
        return out;
    }

    // ---- candidate parsing -------------------------------------------------
    // Everything below writes ONLY staging structures; a throw anywhere
    // leaves the live writer untouched.

    [[nodiscard]] wire::Dict dict_at(const wire::Dict &parent,
                                     const char *key) {
        return wire::section(parent, key);
    }

    [[nodiscard]] std::array<bool, wc::kContactCount>
    parse_enabled(const nb::dict &d, const std::string &path) {
        wire::exact_keys(d, wc::kContactNames, {}, path);
        std::array<bool, wc::kContactCount> out{};
        for (std::size_t i = 0; i < wc::kContactCount; ++i) {
            const char *const name = wc::kContactNames[i].data();
            out[i] = wire::boolean(d[name], path + "." + name);
        }
        return out;
    }

    [[nodiscard]] wc::RiderContactPatchDiag
    parse_patch(nb::handle value, const std::string &path) {
        const nb::dict d = wire::mapping(value, path);
        constexpr auto required =
            wire::keys("in_platform", "normal_load_n", "gap_m", "point_m",
                       "force_on_rider_n", "radial_energy_j",
                       "shear_energy_j");
        wire::exact_keys(d, required, {}, path);
        const auto real = [&d, &path](const char *key) {
            return wire::finite_real(d[key], path + "." + key);
        };
        const auto vec = [&d, &path](const char *key) {
            return wire::fixed<3>(d[key], path + "." + key);
        };
        return {.in_platform =
                    wire::boolean(d["in_platform"], path + ".in_platform"),
                .normal_load_n = real("normal_load_n"),
                .gap_m = real("gap_m"),
                .point_m = vec("point_m"),
                .force_on_rider_n = vec("force_on_rider_n"),
                .radial_energy_j = real("radial_energy_j"),
                .shear_energy_j = real("shear_energy_j")};
    }

    [[nodiscard]] wc::RiderContactSupportDiag
    parse_support_diag(nb::handle value, bool welded,
                       const std::string &path) {
        const nb::dict d = wire::mapping(value, path);
        wc::RiderContactSupportDiag out{};
        out.welded = welded;
        const auto real = [&d, &path](const char *key) {
            return wire::finite_real(d[key], path + "." + key);
        };
        const auto flag = [&d, &path](const char *key) {
            return wire::boolean(d[key], path + "." + key);
        };
        const auto vec = [&d, &path](const char *key) {
            return wire::fixed<3>(d[key], path + "." + key);
        };
        const bool detailed = d.contains("patches");
        if (!detailed) {
            if (welded) {
                constexpr auto base = wire::keys(
                    "enabled", "in_platform", "normal_load_n",
                    "would_separate", "would_slip", "tangent_n", "gap_m",
                    "vertical_force_on_rider_n");
                wire::exact_keys(d, base, {}, path);
            } else {
                constexpr auto base = wire::keys(
                    "enabled", "in_platform", "normal_load_n", "gap_m",
                    "vertical_force_on_rider_n");
                wire::exact_keys(d, base, {}, path);
            }
        } else {
            if (welded) {
                constexpr auto full = wire::keys(
                    "enabled", "in_platform", "normal_load_n",
                    "would_separate", "would_slip", "tangent_n", "gap_m",
                    "vertical_force_on_rider_n", "tangent_force_n",
                    "patches", "force_on_rider_n", "force_on_bike_n",
                    "moment_about_rider_origin_nm", "radial_energy_j",
                    "shear_energy_j", "relative_power_w");
                wire::exact_keys(d, full, {}, path);
            } else {
                constexpr auto full = wire::keys(
                    "enabled", "in_platform", "normal_load_n", "gap_m",
                    "vertical_force_on_rider_n", "tangent_force_n",
                    "patches", "force_on_rider_n", "force_on_bike_n",
                    "moment_about_rider_origin_nm", "radial_energy_j",
                    "shear_energy_j", "relative_power_w");
                wire::exact_keys(d, full, {}, path);
            }
        }
        out.enabled = flag("enabled");
        out.in_platform = flag("in_platform");
        out.normal_load_n = real("normal_load_n");
        if (welded) {
            out.would_separate = flag("would_separate");
            out.would_slip = flag("would_slip");
            out.tangent_n = real("tangent_n");
        }
        out.gap_m = real("gap_m");
        out.vertical_force_on_rider_n = real("vertical_force_on_rider_n");
        out.detailed = detailed;
        if (detailed) {
            if (welded)
                out.tangent_force_vec = vec("tangent_force_n");
            else
                out.tangent_force_scalar = real("tangent_force_n");
            const nb::list patches =
                wire::sequence(d["patches"], path + ".patches");
            std::size_t index = 0;
            for (nb::handle const p : patches)
                out.patches.push_back(
                    parse_patch(p, path + ".patches." +
                                         std::to_string(index++)));
            out.force_on_rider_n = vec("force_on_rider_n");
            out.force_on_bike_n = vec("force_on_bike_n");
            out.moment_nm = vec("moment_about_rider_origin_nm");
            out.radial_energy_j = real("radial_energy_j");
            out.shear_energy_j = real("shear_energy_j");
            out.relative_power_w = real("relative_power_w");
        }
        return out;
    }

    // The spring grip side dict's full field set (the connect layout's
    // extended block minus nothing — spring sides always carry it).
    constexpr auto kGripSideSpringKeys = wire::keys(
        "enabled", "reachable", "overloaded", "trial_pair_force_n",
        "pair_force_limit_n", "release_loss_j", "shoulder_distance_m",
        "arm_reach_m", "hand_gap_m", "point_m", "force_on_rider_n",
        "force_on_bike_n", "elastic_energy_j");
    constexpr auto kGripSpringKeys = wire::keys(
        "enabled", "reachable", "overloaded", "trial_pair_force_n",
        "pair_force_limit_n", "release_loss_j", "shoulder_distance_m",
        "arm_reach_m", "hand_gap_m", "force_on_rider_n", "force_on_bike_n",
        "elastic_energy_j");
    constexpr auto kGripSideWeldedBase = wire::keys("enabled", "reachable",
                                                    "hand_gap_m");
    constexpr auto kGripSideWeldedFull = wire::keys(
        "enabled", "reachable", "hand_gap_m", "overloaded",
        "trial_pair_force_n", "pair_force_limit_n", "release_loss_j",
        "shoulder_distance_m", "arm_reach_m", "point_m", "force_on_rider_n",
        "force_on_bike_n", "elastic_energy_j");
    constexpr auto kGripWeldedFull = wire::keys(
        "enabled", "reachable", "hand_gap_m", "overloaded",
        "trial_pair_force_n", "pair_force_limit_n", "release_loss_j",
        "shoulder_distance_m", "arm_reach_m", "force_on_rider_n",
        "force_on_bike_n", "elastic_energy_j");

    [[nodiscard]] wc::RiderContactGripSideDiag
    parse_grip_side(nb::handle value, bool welded, const std::string &path) {
        const nb::dict d = wire::mapping(value, path);
        wc::RiderContactGripSideDiag out{};
        out.welded = welded;
        const bool extended = !welded || d.contains("overloaded");
        if (welded) {
            if (extended)
                wire::exact_keys(d, kGripSideWeldedFull, {}, path);
            else
                wire::exact_keys(d, kGripSideWeldedBase, {}, path);
        } else {
            wire::exact_keys(d, kGripSideSpringKeys, {}, path);
        }
        const auto real = [&d, &path](const char *key) {
            return wire::finite_real(d[key], path + "." + key);
        };
        const auto flag = [&d, &path](const char *key) {
            return wire::boolean(d[key], path + "." + key);
        };
        out.enabled = flag("enabled");
        out.reachable = flag("reachable");
        out.hand_gap_m = real("hand_gap_m");
        out.extended = extended;
        if (extended) {
            out.overloaded = flag("overloaded");
            out.trial_pair_force_n = real("trial_pair_force_n");
            out.pair_force_limit_n = wire::optional_real(
                d["pair_force_limit_n"], path + ".pair_force_limit_n");
            out.release_loss_j = real("release_loss_j");
            out.shoulder_distance_m = real("shoulder_distance_m");
            out.arm_reach_m = real("arm_reach_m");
            out.point_m = wire::fixed<3>(d["point_m"], path + ".point_m");
            out.force_on_rider_n =
                wire::fixed<3>(d["force_on_rider_n"], path + ".force_on_rider_n");
            out.force_on_bike_n =
                wire::fixed<3>(d["force_on_bike_n"], path + ".force_on_bike_n");
            out.elastic_energy_j = real("elastic_energy_j");
        }
        return out;
    }

    [[nodiscard]] wc::RiderContactGripDiag
    parse_grip(nb::handle value, bool welded, const std::string &path) {
        const nb::dict d = wire::mapping(value, path);
        wc::RiderContactGripDiag out{};
        out.welded = welded;
        const bool extended = !welded || d.contains("overloaded");
        if (welded) {
            if (extended)
                wire::exact_keys(d, kGripWeldedFull, {}, path);
            else
                wire::exact_keys(d, kGripSideWeldedBase, {}, path);
        } else {
            wire::exact_keys(d, kGripSpringKeys, {}, path);
        }
        const auto real = [&d, &path](const char *key) {
            return wire::finite_real(d[key], path + "." + key);
        };
        const auto flag = [&d, &path](const char *key) {
            return wire::boolean(d[key], path + "." + key);
        };
        out.enabled = flag("enabled");
        out.reachable = flag("reachable");
        out.hand_gap_m = real("hand_gap_m");
        out.extended = extended;
        if (extended) {
            out.overloaded = flag("overloaded");
            out.trial_pair_force_n = real("trial_pair_force_n");
            out.pair_force_limit_n = wire::optional_real(
                d["pair_force_limit_n"], path + ".pair_force_limit_n");
            out.release_loss_j = real("release_loss_j");
            out.shoulder_distance_m = real("shoulder_distance_m");
            out.arm_reach_m = real("arm_reach_m");
            out.force_on_rider_n =
                wire::fixed<3>(d["force_on_rider_n"], path + ".force_on_rider_n");
            out.force_on_bike_n =
                wire::fixed<3>(d["force_on_bike_n"], path + ".force_on_bike_n");
            out.elastic_energy_j = real("elastic_energy_j");
        }
        return out;
    }

    // The diagnostics tree the writer's own attachment layout produces —
    // the key sets are config-determined, so a foreign layout is rejected
    // rather than reinterpreted.
    [[nodiscard]] wc::RiderContactsDiagnostics
    parse_diagnostics(nb::handle value, const RiderContactWriter &w,
                      const std::string &path) {
        const nb::dict d = wire::mapping(value, path);
        wc::RiderContactsDiagnostics out{};
        if (d.size() == 0) return out; // the unevaluated {} state
        out.evaluated = true;
        constexpr auto required =
            wire::keys("saddle", "front_pedal", "rear_pedal", "grip_left",
                       "grip_right", "grip");
        wire::exact_keys(d, required, {}, path);
        for (std::size_t si = 0; si < wc::kSupportCount; ++si) {
            const char *const name = wc::kSupportNames[si].data();
            const bool welded =
                si == 0 ? w.linked_saddle() : w.linked_pedals();
            out.supports[si] = parse_support_diag(d[name], welded,
                                                  path + "." + name);
        }
        for (std::size_t side = 0; side < wc::kSideCount; ++side) {
            const std::string name =
                "grip_" + std::string(wc::kSideNames[side]);
            std::string entry_path = path;
            entry_path += '.';
            entry_path += name;
            out.grip_sides[side] =
                parse_grip_side(d[name.c_str()], w.welded_grip(), entry_path);
        }
        out.grip = parse_grip(d["grip"], w.welded_grip(), path + ".grip");
        return out;
    }

    // (welds dict, crank) — the wave-3b latch shape. Entry names must be
    // ones this writer's attachments could have produced.
    [[nodiscard]] std::optional<wc::RiderSettledWelds>
    parse_settled(nb::handle value, const RiderContactWriter &w,
                  const std::string &path) {
        if (value.is_none()) return std::nullopt;
        const nb::list pair = wire::sequence(value, path);
        if (pair.size() != 2)
            wire::invalid(path, "expected the (welds, crank) pair");
        const nb::object welds_raw = pair[0];
        const nb::object crank_raw = pair[1];
        const nb::dict welds =
            wire::mapping(welds_raw, path + ".welds");
        wc::RiderSettledWelds out{};
        for (const auto item : welds) {
            const std::string name = wire::string(item.first, path);
            const std::size_t support_index = [&]() -> std::size_t {
                for (std::size_t i = 0; i < wc::kSupportCount; ++i)
                    if (name == wc::kSupportNames[i]) return i;
                return wc::kSupportCount;
            }();
            const bool is_support =
                support_index < wc::kSupportCount &&
                (support_index == 0 ? w.linked_saddle() : w.linked_pedals());
            const bool is_grip =
                w.welded_grip() && (name == "grip_left" || name == "grip_right");
            std::string entry_path = path;
            entry_path += '.';
            entry_path += name;
            if (!is_support && !is_grip)
                wire::invalid(entry_path, "unknown key");
            const nb::dict entry = wire::mapping(item.second, entry_path);
            wc::RiderSettledEntry parsed{};
            parsed.support_fields = is_support;
            if (is_support)
                wire::exact_keys(entry,
                                 wire::keys("force_on_rider_n", "normal_n",
                                            "tangent_n", "would_separate",
                                            "would_slip"),
                                 {}, entry_path);
            else
                wire::exact_keys(entry, wire::keys("force_on_rider_n"), {},
                                 entry_path);
            parsed.force_on_rider_n = wire::fixed<3>(
                entry["force_on_rider_n"], entry_path + ".force_on_rider_n");
            if (is_support) {
                parsed.normal_n = wire::finite_real(entry["normal_n"],
                                                    entry_path + ".normal_n");
                parsed.tangent_n = wire::finite_real(
                    entry["tangent_n"], entry_path + ".tangent_n");
                parsed.would_separate = wire::boolean(
                    entry["would_separate"], entry_path + ".would_separate");
                parsed.would_slip = wire::boolean(entry["would_slip"],
                                                  entry_path + ".would_slip");
            }
            out.entries.emplace_back(name, parsed);
        }
        out.crank_torque_nm =
            wire::finite_real(crank_raw, path + ".crank_torque_nm");
        return out;
    }

    // The staged state plus the probe copy outsize the 8 KiB frame budget;
    // like drive_binding/attachment_binding this function is parse-only
    // (no audio-safety constraint) so the warning is suppressed locally.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wframe-larger-than")
    [[nodiscard]] std::pair<wc::RiderContactsState,
                          std::optional<wc::RiderContactsProbe>>
    state_from_dict(nb::handle raw, const RiderContactWriter &w) {
        const wire::Dict root{.value = wire::mapping(raw, "rider_contacts_state"),
                              .path = "rider_contacts_state"};
        constexpr auto required = wire::keys(
            "enabled", "supports", "grip_xi_local", "grip_anchor_local",
            "elastic_energy_j", "loss_step_j", "radial_dissipation_power_w",
            "delivered_crank_torque_nm", "last_time_s",
            "pending_release_loss_j", "diagnostics", "settled_welds",
            "last_attachment_samples", "last_attachment_errors",
            "probe_diagnostics", "probe_enabled",
            "probe_delivered_crank_torque_nm");
        wire::exact(root, required);
        wc::RiderContactsState st{};
        const auto real = [&root](const char *key) {
            return wire::finite_real(wire::field(root, key), root.child(key));
        };
        {
            const wire::Dict enabled = dict_at(root, "enabled");
            st.enabled = parse_enabled(enabled.value, enabled.path);
        }
        {
            const wire::Dict supports = dict_at(root, "supports");
            constexpr auto pad_keys =
                wire::keys("saddle:0", "saddle:1", "front_pedal:0",
                           "front_pedal:1", "rear_pedal:0", "rear_pedal:1");
            wire::exact(supports, pad_keys);
            for (std::size_t key = 0; key < wc::kPadStateCount; ++key) {
                const std::string name =
                    std::string(wc::kSupportNames[key / wc::kPadsPerSupport]) +
                    ":" + std::to_string(key % wc::kPadsPerSupport);
                const wire::Dict entry{
                    .value = wire::mapping(supports.value[name.c_str()],
                                           supports.child(name.c_str())),
                    .path = supports.child(name.c_str())};
                wire::exact(entry, wire::keys("xi", "tangent"));
                auto &target = st.supports[key];
                target.xi = wire::finite_real(wire::field(entry, "xi"),
                                              entry.child("xi"));
                const nb::object tangent = wire::field(entry, "tangent");
                target.tangent =
                    tangent.is_none()
                        ? std::nullopt
                        : std::optional<rider::Vec3>(wire::fixed<3>(
                              tangent, entry.child("tangent")));
            }
        }
        // Null-terminated key spellings for the field()/child() readers —
        // the writer's string_view tables are for equality and iteration.
        constexpr std::array<const char *, wc::kSideCount> side_keys = {
            "left", "right"};
        {
            const wire::Dict xi = dict_at(root, "grip_xi_local");
            wire::exact(xi, wire::keys("left", "right"));
            for (std::size_t side = 0; side < wc::kSideCount; ++side)
                st.grip_xi_local[side] = wire::fixed<3>(
                    wire::field(xi, side_keys[side]),
                    xi.child(side_keys[side]));
        }
        {
            const wire::Dict anchors = dict_at(root, "grip_anchor_local");
            wire::exact(anchors, wire::keys("left", "right"));
            for (std::size_t side = 0; side < wc::kSideCount; ++side) {
                const char *const name = side_keys[side];
                const nb::object anchor = wire::field(anchors, name);
                st.grip_anchor_local[side] =
                    anchor.is_none()
                        ? std::nullopt
                        : std::optional<rider::Vec3>(wire::fixed<3>(
                              anchor, anchors.child(name)));
            }
        }
        st.elastic_energy_j = real("elastic_energy_j");
        st.loss_step_j = real("loss_step_j");
        st.radial_dissipation_power_w = real("radial_dissipation_power_w");
        st.delivered_crank_torque_nm = real("delivered_crank_torque_nm");
        st.last_time_s =
            wire::optional_real(wire::field(root, "last_time_s"),
                                root.child("last_time_s"));
        st.pending_release_loss_j = real("pending_release_loss_j");
        st.diagnostics = parse_diagnostics(
            wire::field(root, "diagnostics"), w, root.child("diagnostics"));
        st.settled_welds = parse_settled(
            wire::field(root, "settled_welds"), w, root.child("settled_welds"));
        {
            const wire::Dict samples = dict_at(root, "last_attachment_samples");
            for (const auto item : samples.value) {
                const std::string name = wire::string(item.first, samples.path);
                const std::string entry_path = samples.child(name.c_str());
                const nb::dict entry = wire::mapping(item.second, entry_path);
                constexpr auto sample_keys =
                    wire::keys("kind", "normal_n", "tangent_n", "moment_nm",
                               "gap_m", "pull_n", "half_patch_m");
                wire::exact_keys(entry, sample_keys, {}, entry_path);
                st.attachment_samples.emplace_back(
                    name,
                    rider::AttachmentSample{
                        .kind = wire::string(entry["kind"],
                                             entry_path + ".kind"),
                        .normal_n = wire::finite_real(entry["normal_n"],
                                                      entry_path + ".normal_n"),
                        .tangent_n = wire::finite_real(
                            entry["tangent_n"], entry_path + ".tangent_n"),
                        .moment_nm = wire::finite_real(
                            entry["moment_nm"], entry_path + ".moment_nm"),
                        .gap_m = wire::finite_real(entry["gap_m"],
                                                   entry_path + ".gap_m"),
                        .pull_n = wire::finite_real(entry["pull_n"],
                                                    entry_path + ".pull_n"),
                        .half_patch_m = wire::finite_real(
                            entry["half_patch_m"],
                            entry_path + ".half_patch_m")});
            }
        }
        {
            const nb::list errors = wire::sequence(
                wire::field(root, "last_attachment_errors"),
                root.child("last_attachment_errors"));
            for (nb::handle const item : errors)
                st.attachment_errors.push_back(
                    wire::string(item, root.child("last_attachment_errors")));
        }
        const nb::object probe_diag = wire::field(root, "probe_diagnostics");
        const nb::object probe_enabled = wire::field(root, "probe_enabled");
        const nb::object probe_torque =
            wire::field(root, "probe_delivered_crank_torque_nm");
        const bool any_probe = !probe_diag.is_none() ||
            !probe_enabled.is_none() || !probe_torque.is_none();
        const bool all_probe = !probe_diag.is_none() &&
            !probe_enabled.is_none() && !probe_torque.is_none();
        if (any_probe && !all_probe)
            wire::invalid(root.path, "probe fields must arrive together");
        std::optional<wc::RiderContactsProbe> probe;
        if (all_probe) {
            probe.emplace();
            probe->diagnostics = parse_diagnostics(
                probe_diag, w, root.child("probe_diagnostics"));
            const nb::dict enabled_map = wire::mapping(
                probe_enabled, root.child("probe_enabled"));
            probe->enabled = parse_enabled(enabled_map,
                                           root.child("probe_enabled"));
            probe->delivered_crank_torque_nm = wire::finite_real(
                probe_torque, root.child("probe_delivered_crank_torque_nm"));
        }
        return {std::move(st), std::move(probe)};
    }
    NATIVE_DIAG_POP

    [[nodiscard]] bool truthy(nb::handle value) {
        return PyObject_IsTrue(value.ptr()) == 1;
    }
} // namespace

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void bind_rider_contacts(const nb::module_ &module,
                         nb::class_<Stepper> &cls) {
    static_cast<void>(module);
    // Method face — every call resolves the writer first, so a Stepper
    // built without 'rider_contacts' answers the shared logic_error.
    cls.def("rider_contacts_reset", [](Stepper &s) {
        s.mutate([&] { s.rider_contacts_reset(); });
    });
    cls.def("rider_contacts_restart_clock", [](Stepper &s) {
        s.mutate([&] { s.rider_contacts_restart_clock(); });
    });
    cls.def("rider_contacts_initialize_settled_state", [](Stepper &s) {
        s.mutate([&] { s.rider_contacts_initialize_settled_state(); });
    });
    cls.def("rider_contacts_release_all", [](Stepper &s) {
        s.mutate([&] { s.rider_contacts_release_all(); });
    });
    cls.def("rider_contacts_set_enabled",
            [](Stepper &s, nb::handle name, nb::handle enabled) {
        // set_enabled's gate (rider_contacts.py:126-127): a CONTACTS name
        // plus a literal bool — np.bool_ deliberately does not qualify,
        // like Python's isinstance(enabled, bool).
        const bool valid =
            nb::isinstance<nb::str>(name) &&
            (enabled.ptr() == Py_True || enabled.ptr() == Py_False) &&
            std::ranges::find(wc::kContactNames,
                              nb::cast<std::string>(name)) !=
                wc::kContactNames.end();
        if (!valid)
            throw std::invalid_argument("invalid rider contact enable request");
        return s.mutate([&] {
            return s.rider_contacts_set_enabled(nb::cast<std::string>(name),
                                                enabled.ptr() == Py_True);
        });
    }, nb::arg("name"), nb::arg("enabled"));
    cls.def("rider_contacts_qfrc",
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) named nb::args mirror the oracle's keyword order
            [](Stepper &s, nb::handle dt, nb::handle advance,
               nb::handle detailed) {
        // wire::real keeps NaN/inf for the writer's positive-domain gate —
        // 'invalid rider contact dt' covers nonfinite and nonpositive
        // alike, like scalar(dt, ..., positive=True).
        const double checked = wire::real(dt, "rider_contacts_qfrc.dt");
        const bool advancing = truthy(advance);
        const bool detail = truthy(detailed);
        return s.mutate([&] {
            return wire::owned_array<double>(
                s.rider_contacts_qfrc(checked, advancing, detail));
        });
    // .none() on dt: nanobind rejects None for object-typed args at
    // dispatch; the oracle's scalar(None,...) instead surfaces ValueError
    // from wire::real like every other non-real substitution.
    }, nb::arg("dt").none(), nb::arg("advance") = true,
       nb::arg("detailed") = true);
    cls.def("rider_contacts_stored_energy", [](Stepper &s) {
        return s.rider_contacts_stored_energy();
    });
    cls.def("rider_contacts_diagnostics",
            [](Stepper &s, nb::handle probe) {
        const RiderContactWriter &w = s.rider_contacts();
        if (truthy(probe)) {
            if (!w.probe().has_value()) return nb::object(nb::none());
            return nb::object(diagnostics_dict(w.probe()->diagnostics));
        }
        return nb::object(diagnostics_dict(w.state().diagnostics));
    }, nb::arg("probe") = false);
    cls.def("rider_contacts_state", [](Stepper &s) {
        const RiderContactWriter &w = s.rider_contacts();
        return state_dict(w.state(), w.probe());
    });
    cls.def("set_rider_contacts_state", [](Stepper &s, nb::handle raw) {
        RiderContactWriter &w = s.rider_contacts();
        auto [state, probe] = state_from_dict(raw, w);
        s.mutate([&] {
            w.assign_state(std::move(state), std::move(probe));
        });
    }, nb::arg("state").none());
}
NATIVE_DIAG_POP
