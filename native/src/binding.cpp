// binding.cpp — nanobind FFI edge for the native Stepper.
//
// qpos/qvel/qacc/qfrc_constraint/ctrl (and qfrc_applied/actuator_force
// in drive_binding.cpp) return zero-copy read-only numpy views onto
// mjData's FIXED buffers — their addresses are stable for the mjData
// lifetime. The declared owner policy is nb::rv_policy::reference_internal:
// the array's base retains the Stepper, so a view can never outlive its
// model/data. Views are LIVE windows — contents change with each
// forward/step — re-read the property rather than caching the array.
// efc_force is the exception: d->efc_force is a per-frame constraint
// arena allocation whose address and length move whenever the contact
// set changes, so the property returns an OWNED snapshot copy instead
// (writeable; frozen at access time — see the prop for the policy).
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <exception>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <vector>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include "stepper.hpp"
#include "config.hpp"
#include "engaged.hpp"
#include "numeric_numpy.hpp"
#include "pneumatic/brush.hpp"
#include "pneumatic/carcass.hpp"
#include "pneumatic/geometry.hpp"
#include "pneumatic/model.hpp"
#include "pneumatic/profile.hpp"
#include "tyre/distance.hpp"
#include "binding_readers.hpp"
#include "binding_arrays.hpp"
#include "engine_call.hpp"
#include "diag.hpp"
#include "rider/attachment_binding.hpp"
#include "rider/contact_binding.hpp"
#include "rider/rider_contact_binding.hpp"
#include "drivetrain/policy_binding.hpp"
#include "drivetrain/drive_binding.hpp"
#include "runtime/runtime_binding.hpp"
#include "runtime/research_binding.hpp"
#include "runtime/provenance.hpp"
#include "runtime/test_adapter.hpp"
#include "writers/cruise.hpp"
#include "writers/resistance.hpp"
// The binding speaks the shared tyre value types; backend dispatch lives
// behind Stepper::tire_writer()'s view.
#include "writers/tire_common.hpp"

namespace nb = nanobind;

namespace {
    void translate_engine_failure(const std::exception_ptr &exception,
                                  void *python_class) {
        try {
            std::rethrow_exception(exception);
        } catch (const engine::EngineFailure &failure) {
            // A stock MuJoCo Python callback can set its original Python
            // error before raising mju_error. Keep that exception intact.
            if (!PyErr_Occurred())
                PyErr_SetString(static_cast<PyObject *>(python_class), failure.what());
        }
    }

    // Read-only numpy view over a Stepper span — shared by every view
    // prop_ro. The empty nb::handle() owner pairs with the declared
    // nb::rv_policy::reference_internal at each def site, which makes the
    // Stepper the array's owner (the view keeps its Stepper alive).
    nb::ndarray<nb::numpy, const double, nb::shape<-1> >
    as_view(std::span<const double> v) {
        return nb::ndarray<nb::numpy, const double, nb::shape<-1> >(
            v.data(), {v.size()}, nb::handle());
    }

    nb::ndarray<nb::numpy, double, nb::shape<-1>> as_owned(std::vector<double> values) {
        return wire::owned_array<double>(values);
    }

    nb::ndarray<nb::numpy, bool, nb::shape<-1>> as_owned_bool(std::vector<char> values) {
        return wire::owned_flags(values);
    }

    // One TireSnapshot as a dict carrying BOTH consumers' schemas: the
    // manifest's readable digest keys (what the artifact serializes in
    // tools/golden_episode._tire_snapshot_row) and the flat-array keys
    // resistance_components's side_input reads.
    nb::dict snapshot_dict(const TireSnapshot &s) {
        nb::dict d;
        d["time_s"] = s.time_s;
        d["interval_id"] = s.interval_id;
        d["backend"] = s.backend;
        d["geometric_contact"] = s.geometric_contact;
        d["effective_radius_m"] = s.effective_radius_m;
        std::vector<double> loads;
        std::vector<char> working;
        loads.reserve(s.patches.size());
        working.reserve(s.patches.size());
        nb::list patches;
        for (const TirePatch &p: s.patches) {
            loads.push_back(p.normal_load_n);
            working.push_back(static_cast<char>(p.working_surface));
            nb::dict pd;
            pd["normal_load_n"] = p.normal_load_n;
            pd["tangent_force_n"] = p.tangent_force_n;
            pd["slip_mps"] = p.slip_mps;
            pd["working_surface"] = p.working_surface;
            patches.append(pd);
        }
        d["patches"] = std::move(patches);
        d["patch_loads"] = as_owned(std::move(loads));
        d["patch_working"] = as_owned_bool(std::move(working));
        d["eff_radius"] = s.effective_radius_m;
        return d;
    }

    // One tire-snapshot side, flat-array schema:
    // {'patch_loads': f64[n], 'patch_working': bool[n], 'eff_radius': float}.
    // Dtype and layout conversions are accepted. Keep their ndarray owners alive
    // through the writer call: a span alone cannot retain a conversion temporary.
    struct OwnedTireSideInput {
        nb::ndarray<const double, nb::shape<-1>, nb::c_contig> loads;
        nb::ndarray<const bool, nb::shape<-1>, nb::c_contig> working;
        double effective_radius_m;

        [[nodiscard]] TireSideInput view() const {
            return {
                .patch_loads = std::views::counted(loads.data(),
                                                   static_cast<std::ptrdiff_t>(loads.size())),
                .patch_working = std::views::counted(working.data(),
                                                     static_cast<std::ptrdiff_t>(working.size())),
                .effective_radius_m = effective_radius_m
            };
        }
    };

    // Owned (N,2) float64 array — the 2-D sibling of wire::owned_array for
    // the _debug_signed_distance test surface, keeping the same
    // capsule-owner discipline (the ndarray owns the released buffer).
    nb::ndarray<nb::numpy, double, nb::shape<-1, -1>>
    as_owned_grid(std::vector<double> values) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) ndarray requires runtime-sized contiguous primitive storage
        auto storage = std::make_unique<double[]>(values.size());
        std::ranges::copy(values, storage.get());
        auto [data, owner] = wire::detail::release_with_owner(
            std::move(storage), [](double *value) {
                return nb::capsule(value, [](void *p) noexcept {
                    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) capsule owns the released unique_ptr array
                    delete[] static_cast<double *>(p);
                });
            });
        return {data, {values.size() / 2, 2}, owner};
    }

    // Owned (K,2) int32 array — the same capsule-owner discipline as
    // as_owned_grid, for the _debug_pneumatic_patches surface's index
    // pairs.
    nb::ndarray<nb::numpy, std::int32_t, nb::shape<-1, -1>>
    as_owned_pairs32(std::span<const std::pair<int, int>> pairs) {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) ndarray requires runtime-sized contiguous primitive storage
        auto storage = std::make_unique<std::int32_t[]>(2 * pairs.size());
        const std::span<std::int32_t> destination = std::views::counted(
            storage.get(),
            static_cast<std::ptrdiff_t>(2 * pairs.size()));
        for (std::size_t i = 0; i < pairs.size(); ++i) {
            destination[2 * i] = static_cast<std::int32_t>(pairs[i].first);
            destination[2 * i + 1] =
                static_cast<std::int32_t>(pairs[i].second);
        }
        auto [data, owner] = wire::detail::release_with_owner(
            std::move(storage), [](std::int32_t *value) {
                return nb::capsule(value, [](void *p) noexcept {
                    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory) capsule owns the released unique_ptr array
                    delete[] static_cast<std::int32_t *>(p);
                });
            });
        return {data, {pairs.size(), 2}, owner};
    }

    // --- Pneumatic carcass debug-surface intake ---------------------
    // Contiguous float64 row plus the TyreSpecs / RayHits /
    // CarcassState dict projections the _debug_pneumatic_carcass*
    // surfaces share. State dicts omit n_rays — the shared row length
    // stands in for it (the kernel's own state-vs-ring check then
    // fires exactly like the oracle's).

    nb::ndarray<const double, nb::shape<-1>, nb::c_contig>
    debug_carcass_row64(nb::handle value, std::string_view path) {
        try {
            return nb::cast<nb::ndarray<const double, nb::shape<-1>,
                                        nb::c_contig>>(value);
        } catch (const nb::cast_error &) {
            wire::invalid(path, "expected a contiguous float64 array");
        }
    }

    std::span<const double> debug_carcass_span(
            const nb::ndarray<const double, nb::shape<-1>,
                              nb::c_contig> &row) noexcept {
        return std::views::counted(
                row.data(), static_cast<std::ptrdiff_t>(row.shape(0)));
    }

    std::vector<double> debug_carcass_row_vec(const wire::Dict &d,
                                              const char *key) {
        const auto row = debug_carcass_row64(d[key], d.child(key));
        const std::span<const double> s = debug_carcass_span(row);
        return {s.begin(), s.end()};
    }

    // The 18 TyreSpecs fields (physics/tyre.py:64-78) — wire::real
    // keeps the oracle's domain (TyreSpecs.__post_init__ only bounds
    // signs: an inf passes `> 0`, and the kernel would propagate it).
    nativecfg::PneumaticTyreSpec debug_carcass_spec(nb::handle raw) {
        const wire::Dict spec{.value = wire::mapping(raw, "spec"),
                              .path = "spec"};
        constexpr auto required = wire::keys(
                "name", "outer_radius_mm", "rim_radius_mm",
                "section_height_mm", "pressure_bar", "crr_reference",
                "casing_width_mm", "compressed_casing_mm", "area_factor",
                "carcass_stiffness_n_mm2", "contact_length_factor",
                "rate_stiffening", "rate_relaxation_s", "loss_factor",
                "hysteresis_rate_eps_mps", "relaxation_length_mm",
                "rim_stiffness_n_mm2", "tread_loss_crr");
        wire::exact(spec, required);
        nativecfg::PneumaticTyreSpec t;
        t.name = wire::string(spec["name"], spec.child("name"));
        t.outer_radius_mm =
                wire::real(spec["outer_radius_mm"], "spec.outer_radius_mm");
        t.rim_radius_mm =
                wire::real(spec["rim_radius_mm"], "spec.rim_radius_mm");
        t.section_height_mm = wire::real(spec["section_height_mm"],
                                         "spec.section_height_mm");
        t.pressure_bar =
                wire::real(spec["pressure_bar"], "spec.pressure_bar");
        t.crr_reference =
                wire::real(spec["crr_reference"], "spec.crr_reference");
        t.casing_width_mm = wire::real(spec["casing_width_mm"],
                                       "spec.casing_width_mm");
        t.compressed_casing_mm = wire::real(spec["compressed_casing_mm"],
                                            "spec.compressed_casing_mm");
        t.area_factor =
                wire::real(spec["area_factor"], "spec.area_factor");
        t.carcass_stiffness_n_mm2 = wire::real(
                spec["carcass_stiffness_n_mm2"],
                "spec.carcass_stiffness_n_mm2");
        t.contact_length_factor = wire::real(
                spec["contact_length_factor"], "spec.contact_length_factor");
        t.rate_stiffening =
                wire::real(spec["rate_stiffening"], "spec.rate_stiffening");
        t.rate_relaxation_s = wire::real(spec["rate_relaxation_s"],
                                         "spec.rate_relaxation_s");
        t.loss_factor =
                wire::real(spec["loss_factor"], "spec.loss_factor");
        t.hysteresis_rate_eps_mps = wire::real(
                spec["hysteresis_rate_eps_mps"],
                "spec.hysteresis_rate_eps_mps");
        t.relaxation_length_mm = wire::real(spec["relaxation_length_mm"],
                                            "spec.relaxation_length_mm");
        t.rim_stiffness_n_mm2 = wire::real(
                spec["rim_stiffness_n_mm2"], "spec.rim_stiffness_n_mm2");
        t.tread_loss_crr = wire::real(spec["tread_loss_crr"],
                                      "spec.tread_loss_crr");
        return t;
    }

    biketyre::CarcassState debug_carcass_state(nb::handle raw) {
        const wire::Dict state{.value = wire::mapping(raw, "state"),
                               .path = "state"};
        constexpr auto required = wire::keys(
                "delta_prev_m", "elastic_prev_n", "maxwell_force_n",
                "initialized", "rim_strike_active", "rim_event_x_m",
                "rim_event_speed_mps", "rim_event_peak_load_n",
                "rim_event_peak_force_n", "rim_event_energy_j");
        wire::exact(state, required);
        const auto delta = debug_carcass_row64(state["delta_prev_m"],
                                               "state.delta_prev_m");
        const auto elastic = debug_carcass_row64(state["elastic_prev_n"],
                                                 "state.elastic_prev_n");
        const auto maxwell = debug_carcass_row64(state["maxwell_force_n"],
                                                 "state.maxwell_force_n");
        if (elastic.shape(0) != delta.shape(0) ||
            maxwell.shape(0) != delta.shape(0))
            wire::invalid("state", "history rows must share one width");
        biketyre::CarcassState s(
                static_cast<std::int64_t>(delta.shape(0)));
        const auto d = debug_carcass_span(delta);
        std::ranges::copy(d, s.delta_prev_m().begin());
        const auto e = debug_carcass_span(elastic);
        std::ranges::copy(e, s.elastic_prev_n().begin());
        const auto v = debug_carcass_span(maxwell);
        std::ranges::copy(v, s.maxwell_force_n().begin());
        s.restore_fields(
                wire::boolean(state["initialized"], "state.initialized"),
                wire::boolean(state["rim_strike_active"],
                              "state.rim_strike_active"),
                wire::real(state["rim_event_x_m"], "state.rim_event_x_m"),
                wire::real(state["rim_event_speed_mps"],
                           "state.rim_event_speed_mps"),
                wire::real(state["rim_event_peak_load_n"],
                           "state.rim_event_peak_load_n"),
                wire::real(state["rim_event_peak_force_n"],
                           "state.rim_event_peak_force_n"),
                wire::real(state["rim_event_energy_j"],
                           "state.rim_event_energy_j"));
        return s;
    }

    nb::dict debug_carcass_state_dict(const biketyre::CarcassState &s) {
        nb::dict d;
        d["delta_prev_m"] = wire::owned_array<double>(s.delta_prev_m());
        d["elastic_prev_n"] =
                wire::owned_array<double>(s.elastic_prev_n());
        d["maxwell_force_n"] =
                wire::owned_array<double>(s.maxwell_force_n());
        d["initialized"] = s.initialized();
        d["rim_strike_active"] = s.rim_strike_active();
        d["rim_event_x_m"] = s.rim_event_x_m();
        d["rim_event_speed_mps"] = s.rim_event_speed_mps();
        d["rim_event_peak_load_n"] = s.rim_event_peak_load_n();
        d["rim_event_peak_force_n"] = s.rim_event_peak_force_n();
        d["rim_event_energy_j"] = s.rim_event_energy_j();
        return d;
    }

    // --- Pneumatic brush debug-surface intake ----------------------
    // The SurfaceSpec dict projection (terrain/surface.py:31-53) the
    // brush surfaces share, plus the DiscretisedBrush state dict
    // (bristle row + kappa_prime). Fields use wire::real — the kernels'
    // own validation decides the domain, like the oracle's SurfaceSpec
    // / brush.py checks.

    nativecfg::SurfaceSpec debug_brush_surface(nb::handle raw) {
        const wire::Dict surface{.value = wire::mapping(raw, "surface"),
                                 .path = "surface"};
        constexpr auto required = wire::keys(
                "name", "mu_peak", "mu_slide",
                "slip_stiffness_per_load", "stribeck_speed_mps");
        wire::exact(surface, required);
        nativecfg::SurfaceSpec s;
        s.name = wire::string(surface["name"], surface.child("name"));
        s.mu_peak =
                wire::real(surface["mu_peak"], "surface.mu_peak");
        s.mu_slide =
                wire::real(surface["mu_slide"], "surface.mu_slide");
        s.slip_stiffness_per_load =
                wire::real(surface["slip_stiffness_per_load"],
                           "surface.slip_stiffness_per_load");
        s.stribeck_speed_mps =
                wire::real(surface["stribeck_speed_mps"],
                           "surface.stribeck_speed_mps");
        return s;
    }

    biketyre::DiscretisedBrush debug_brush_state(nb::handle raw) {
        const wire::Dict state{.value = wire::mapping(raw, "state"),
                               .path = "state"};
        constexpr auto required =
                wire::keys("bristle_deflection_m", "kappa_prime");
        wire::exact(state, required);
        const auto row = debug_carcass_row64(
                state["bristle_deflection_m"],
                "state.bristle_deflection_m");
        const std::span<const double> bristle = debug_carcass_span(row);
        biketyre::DiscretisedBrush brush(
                static_cast<std::int64_t>(bristle.size()));
        std::ranges::copy(bristle,
                          brush.bristle_deflection_m().begin());
        brush.restore_kappa_prime(
                wire::real(state["kappa_prime"], "state.kappa_prime"));
        return brush;
    }

    nb::dict debug_brush_state_dict(const biketyre::DiscretisedBrush &b) {
        nb::dict d;
        d["bristle_deflection_m"] =
                wire::owned_array<double>(b.bristle_deflection_m());
        d["kappa_prime"] = b.kappa_prime();
        return d;
    }

    nb::dict debug_brush_result_dict(const biketyre::BrushResult &r) {
        nb::dict d;
        d["force_n"] = r.force_n;
        d["fully_sliding"] = r.fully_sliding;
        d["friction_coefficient"] = r.friction_coefficient;
        return d;
    }

    // Optional tread_stiffness_n_m2 — None maps to std::nullopt; the
    // value itself stays wire::real so the kernel's own finite/nonneg
    // check fires like the oracle's (brush.py:82-83).
    std::optional<double> debug_optional_tread(nb::handle raw) {
        if (raw.is_none()) return std::nullopt;
        return wire::real(raw, "tread_stiffness_n_m2");
    }

    // --- Pneumatic model debug-surface intake ----------------------
    // The {x0_m, dx_m, z_m} road profile (the native/config.py
    // pneumatic.profile projection — the same keys
    // _debug_pneumatic_intersect reads), the {surface, sections}
    // SurfaceMap wire (tire.surface_map — config.hpp:180-191), the
    // tier dict's three model-consumed fields (name/timestep_s ride
    // along unused like the oracle's TierSpec), and the
    // {carcass, patch_states} semantic-state projection — patch state
    // entries carry the oracle's _PatchBrushState fields with
    // `detailed` as None-or-{bristle_deflection_m, kappa_prime}.

    biketyre::RoadProfile debug_model_profile(nb::handle raw) {
        const wire::Dict profile{.value = wire::mapping(raw, "profile"),
                                 .path = "profile"};
        wire::exact(profile, wire::keys("x0_m", "dx_m", "z_m"));
        const auto z_wire = debug_carcass_row64(profile["z_m"],
                                                "profile.z_m");
        const std::span<const double> z_span =
                debug_carcass_span(z_wire);
        return {
                wire::finite_real(profile["x0_m"], "profile.x0_m"),
                wire::finite_real(profile["dx_m"], "profile.dx_m"),
                {z_span.begin(), z_span.end()}};
    }

    nativecfg::SurfaceMap debug_model_surface_map(nb::handle raw) {
        const wire::Dict map{.value = wire::mapping(raw, "surface_map"),
                             .path = "surface_map"};
        wire::exact(map, wire::keys("surface", "sections"));
        nativecfg::SurfaceMap out;
        out.surface = debug_brush_surface(map["surface"]);
        const nb::list sections =
                wire::sequence(map["sections"], "surface_map.sections");
        out.sections.reserve(sections.size());
        for (nb::handle const item: sections) {
            const wire::Dict section{
                    .value = wire::mapping(item,
                                           "surface_map.sections"),
                    .path = "surface_map.sections"};
            wire::exact(section,
                        wire::keys("start_m", "end_m", "surface"));
            out.sections.push_back(nativecfg::SurfaceSection{
                    .start_m = wire::real(section["start_m"],
                                          "surface_map.sections."
                                          "start_m"),
                    .end_m = wire::real(section["end_m"],
                                        "surface_map.sections.end_m"),
                    .surface = debug_brush_surface(
                            section["surface"])});
        }
        return out;
    }

    // {n_rays, half_angle_deg, discretised_brush} — the tier fields
    // PneumaticTyre consumes (model.py:160-164); name/timestep_s ride
    // along unused exactly like the oracle's stored TierSpec.
    struct DebugModelTier {
        std::int64_t n_rays;
        double half_angle_deg;
        bool discretised_brush;
    };

    DebugModelTier debug_model_tier(nb::handle raw) {
        const wire::Dict tier{.value = wire::mapping(raw, "tier"),
                              .path = "tier"};
        wire::exact(tier,
                    wire::keys("n_rays", "half_angle_deg",
                               "discretised_brush"),
                    wire::keys("name", "timestep_s"));
        return {.n_rays =
                        wire::integer(tier["n_rays"], "tier.n_rays"),
                .half_angle_deg = wire::real(tier["half_angle_deg"],
                                             "tier.half_angle_deg"),
                .discretised_brush = wire::boolean(
                        tier["discretised_brush"],
                        "tier.discretised_brush")};
    }

    // The _PatchBrushState dict projection: {start, stop, kappa_prime,
    // detailed} where detailed is None or {bristle_deflection_m,
    // kappa_prime}. Row length is the detailed brush's n_elements —
    // it may differ from (stop - start) under injection, exactly like
    // the oracle's free-standing DiscretisedBrush objects.
    biketyre::PatchBrushState debug_model_patch_state(nb::handle raw) {
        const wire::Dict entry{.value = wire::mapping(
                                       raw,
                                       "state.patch_states"),
                               .path = "state.patch_states"};
        wire::exact(entry, wire::keys("start", "stop", "kappa_prime",
                                      "detailed"));
        biketyre::PatchBrushState s;
        s.start = wire::integer32(entry["start"],
                                  "state.patch_states.start");
        s.stop = wire::integer32(entry["stop"],
                                 "state.patch_states.stop");
        s.kappa_prime =
                wire::real(entry["kappa_prime"],
                           "state.patch_states.kappa_prime");
        nb::handle const detailed = entry["detailed"];
        if (detailed.is_none()) {
            s.has_detailed = false;
            return s;
        }
        const wire::Dict d{.value = wire::mapping(
                                   detailed,
                                   "state.patch_states.detailed"),
                           .path = "state.patch_states.detailed"};
        wire::exact(d, wire::keys("bristle_deflection_m",
                                  "kappa_prime"));
        const auto row = debug_carcass_row64(
                d["bristle_deflection_m"],
                "state.patch_states.detailed.bristle_deflection_m");
        const std::span<const double> bristle =
                debug_carcass_span(row);
        s.has_detailed = true;
        s.detailed_kappa_prime = wire::real(
                d["kappa_prime"],
                "state.patch_states.detailed.kappa_prime");
        s.bristle_deflection_m.assign(bristle.begin(), bristle.end());
        return s;
    }

    // The {carcass, patch_states} semantic-state intake — carcass
    // rows are checked against the ring inside assign(), like the
    // oracle's state-vs-ring gate; the patch pool validates bristle
    // capacity.
    void debug_model_restore_state(const nb::dict &raw,
                                   biketyre::PneumaticTyreModel &model) {
        const wire::Dict state{.value = raw, .path = "state"};
        wire::exact(state, wire::keys("carcass", "patch_states"));
        model.carcass_state().assign(
                debug_carcass_state(state["carcass"]));
        const nb::list slots = wire::sequence(
                state["patch_states"], "state.patch_states");
        std::vector<biketyre::PatchBrushState> states;
        states.reserve(slots.size());
        for (nb::handle const item: slots)
            states.push_back(debug_model_patch_state(item));
        model.restore_patch_states(states);
    }

    nb::dict debug_model_patch_state_dict(
            const biketyre::PatchBrushState &s) {
        nb::dict d;
        d["start"] = s.start;
        d["stop"] = s.stop;
        d["kappa_prime"] = s.kappa_prime;
        if (s.has_detailed) {
            nb::dict det;
            det["bristle_deflection_m"] = wire::owned_array<double>(
                    s.bristle_deflection_m);
            det["kappa_prime"] = s.detailed_kappa_prime;
            d["detailed"] = det;
        } else {
            d["detailed"] = nb::none();
        }
        return d;
    }

    nb::dict debug_model_state_dict(
            const biketyre::PneumaticTyreModel &model) {
        nb::dict d;
        d["carcass"] = debug_carcass_state_dict(model.carcass_state());
        nb::list slots;
        for (const biketyre::PatchBrushState &s:
             model.patch_states())
            slots.append(debug_model_patch_state_dict(s));
        d["patch_states"] = std::move(slots);
        return d;
    }

    nb::dict debug_model_rim_event_dict(
            const biketyre::RimStrikeEvent &ev) {
        nb::dict d;
        d["x_m"] = ev.x_m;
        d["speed_mps"] = ev.speed_mps;
        d["peak_load_n"] = ev.peak_load_n;
        d["peak_rim_force_n"] = ev.peak_rim_force_n;
        d["absorbed_energy_j"] = ev.absorbed_energy_j;
        return d;
    }

    // Every TyrePatchOutput / WheelOutputs field (model.py:34-108) —
    // fixed 3-arrays stand in for the oracle's (3,) ndarrays.
    nb::dict debug_model_patch_dict(
            const biketyre::TyrePatchOutput &p) {
        nb::dict d;
        d["ray_start"] = p.ray_start;
        d["ray_stop"] = p.ray_stop;
        d["surface_name"] = p.surface_name;
        d["normal_load_n"] = p.normal_load_n;
        d["support_n"] = p.support_n;
        d["tangential_force_n"] = p.tangential_force_n;
        d["normal_force_world_n"] =
                wire::owned_array<double>(p.normal_force_world_n);
        d["tangent_world"] =
                wire::owned_array<double>(p.tangent_world);
        d["force_world_n"] =
                wire::owned_array<double>(p.force_world_n);
        d["centroid_world_m"] =
                wire::owned_array<double>(p.centroid_world_m);
        d["mean_deflection_m"] = p.mean_deflection_m;
        d["contact_length_m"] = p.contact_length_m;
        d["slip_ratio"] = p.slip_ratio;
        d["transient_slip_ratio"] = p.transient_slip_ratio;
        d["sliding_speed_mps"] = p.sliding_speed_mps;
        d["slip_speed_mps"] = p.slip_speed_mps;
        d["friction_coefficient"] = p.friction_coefficient;
        d["fully_sliding"] = p.fully_sliding;
        d["dissipated_power_w"] = p.dissipated_power_w;
        return d;
    }

    nb::dict debug_model_outputs_dict(
            const biketyre::WheelOutputs &out) {
        nb::dict d;
        d["tyre_name"] = out.tyre_name;
        d["pressure_bar"] = out.pressure_bar;
        d["hub_position_world_m"] =
                wire::owned_array<double>(out.hub_position_world_m);
        d["hub_velocity_world_mps"] =
                wire::owned_array<double>(out.hub_velocity_world_mps);
        d["omega_forward_radps"] = out.omega_forward_radps;
        d["force_world_n"] =
                wire::owned_array<double>(out.force_world_n);
        d["normal_load_n"] = out.normal_load_n;
        d["support_n"] = out.support_n;
        d["mean_deflection_m"] = out.mean_deflection_m;
        d["contact_length_m"] = out.contact_length_m;
        d["slip_ratio"] = out.slip_ratio;
        d["transient_slip_ratio"] = out.transient_slip_ratio;
        d["fully_sliding"] = out.fully_sliding;
        d["rim_strike_active"] = out.rim_strike_active;
        if (out.rim_event.has_value())
            d["rim_event"] = debug_model_rim_event_dict(*out.rim_event);
        else
            d["rim_event"] = nb::none();
        d["dissipated_power_w"] = out.dissipated_power_w;
        d["coverage_event"] = out.coverage_event;
        d["airborne"] = out.airborne;
        nb::list patches;
        for (const biketyre::TyrePatchOutput &p: out.patches)
            patches.append(debug_model_patch_dict(p));
        d["patches"] = std::move(patches);
        return d;
    }

    OwnedTireSideInput side_input(const nb::dict &snaps, const char *side) {
        if (!snaps.contains(side))
            throw std::invalid_argument(
                "resistance_components: missing side '" + std::string(side) +
                "'");
        const wire::Dict root{.value = snaps, .path = "resistance_components"};
        const auto parsed = wire::section(root, side);
        constexpr auto required = wire::keys("patch_loads", "patch_working", "eff_radius");
        constexpr auto optional = wire::keys("time_s", "interval_id", "backend", "geometric_contact", "effective_radius_m", "patches");
        wire::exact(parsed, required, optional);
        if (parsed.contains("patches")) {
            const auto patches = wire::sequence(parsed["patches"], parsed.child("patches"));
            std::size_t index = 0;
            for (nb::handle const value: patches) {
                const std::string patch_path = parsed.child("patches") + "." + std::to_string(index++);
                const wire::Dict patch{.value = wire::mapping(value, patch_path), .path = patch_path};
                wire::exact(patch, wire::keys("normal_load_n", "tangent_force_n", "slip_mps", "working_surface"));
                for (const char *key: {"normal_load_n", "tangent_force_n", "slip_mps"})
                    static_cast<void>(wire::finite_real(patch[key], patch.child(key)));
                static_cast<void>(wire::boolean(patch["working_surface"], patch.child("working_surface")));
            }
        }
        for (const char *key: {"time_s", "effective_radius_m"})
            if (parsed.contains(key)) static_cast<void>(wire::finite_real(parsed[key], parsed.child(key)));
        if (parsed.contains("interval_id")) static_cast<void>(wire::integer(parsed["interval_id"], parsed.child("interval_id")));
        if (parsed.contains("backend")) static_cast<void>(wire::string(parsed["backend"], parsed.child("backend")));
        if (parsed.contains("geometric_contact")) static_cast<void>(wire::boolean(parsed["geometric_contact"], parsed.child("geometric_contact")));
        const nb::dict d = parsed.value;
        const auto arr = [&d, side](const char *key) -> nb::object {
            if (!d.contains(key))
                throw std::invalid_argument(
                    "resistance_components: '" + std::string(side) + "." + key +
                    "' is required");
            return d[key];
        };
        using F64 = nb::ndarray<const double, nb::shape<-1>, nb::c_contig>;
        using B1 = nb::ndarray<const bool, nb::shape<-1>, nb::c_contig>;
        const auto raw_loads = arr("patch_loads");
        static_cast<void>(wire::vector(raw_loads, parsed.child("patch_loads")));
        F64 loads;
        try { loads = nb::cast<F64>(raw_loads); }
        catch (const nb::cast_error &) { wire::invalid(parsed.child("patch_loads"), "expected 1-D numeric array"); }
        B1 working;
        try { working = nb::cast<B1>(arr("patch_working")); }
        catch (const nb::cast_error &) { wire::invalid(parsed.child("patch_working"), "expected 1-D flag array"); }
        return {.loads = loads, .working = working, .effective_radius_m = wire::finite_real(arr("eff_radius"), parsed.child("eff_radius"))};
    }

    // Debug-only oracle surfaces for the whole-wheel pneumatic model
    // — used by tests/reference/test_native_pneumatic_model.py to run
    // identical multi-step sequences through
    // biketyre::PneumaticTyreModel and sim/ride/tyre/model.py
    // PneumaticTyre. The spec dict is the 18-field TyreSpecs wire; the
    // tier dict carries the three consumed fields (name/timestep_s
    // ride along unused); profile is {x0_m, dx_m, z_m}; surface_map is
    // the {surface, sections} tire.surface_map wire; state is
    // {carcass, patch_states}; step carries the per-call inputs —
    // {hub, velocity, omega_forward_radps, dt_s} plus an optional
    // {pressure} — so the bound signature stays under nine args
    // (g++-16 -flto ICEs in the final RTL pass emitting the nanobind
    // trampoline for a 9+ arg def in this TU). hub/velocity arrive as
    // contiguous f64 rows the model itself length-checks (the
    // oracle's ndarray shape gate), and the scalars go through
    // wire::real so NaN/inf reach the model's own validation in
    // oracle order. Returned is every WheelOutputs field — patches as
    // a list of the 19 TyrePatchOutput fields — plus the post-call
    // state.
    // Kept out of NB_MODULE's body: the module-init function is large
    // enough that g++-16 -flto ICEs in the final RTL pass when these
    // lambdas are inlined into it. noinline keeps each trampoline out
    // of the exec_impl ltrans partition too.
    [[gnu::noinline]] void
    bind_pneumatic_evaluate_debug(nb::module_ &m) {
        m.def("_debug_pneumatic_evaluate",
              // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python surface signature
              [](nb::handle raw_spec, nb::handle raw_tier,
                 nb::handle raw_profile, nb::handle raw_surface_map,
                 nb::handle raw_state, nb::handle raw_step) {
        const nativecfg::PneumaticTyreSpec spec =
                debug_carcass_spec(raw_spec);
        const DebugModelTier tier = debug_model_tier(raw_tier);
        biketyre::PneumaticTyreModel model(
                spec, tier.n_rays, tier.half_angle_deg,
                tier.discretised_brush);
        const biketyre::RoadProfile road =
                debug_model_profile(raw_profile);
        const nativecfg::SurfaceMap map =
                debug_model_surface_map(raw_surface_map);
        debug_model_restore_state(wire::mapping(raw_state, "state"),
                                  model);
        const wire::Dict step{
                .value = wire::mapping(raw_step, "step"),
                .path = "step"};
        wire::exact(step,
                    wire::keys("hub", "velocity",
                               "omega_forward_radps", "dt_s"),
                    wire::keys("pressure"));
        // model.py:173-175 — an optional set_pressure call applied
        // before the step, exercising the with_pressure mutation path
        // (clamped write into the live spec).
        if (step.contains("pressure") &&
            !step["pressure"].is_none())
            model.set_pressure(
                    wire::real(step["pressure"], "step.pressure"));
        const auto hub = debug_carcass_row64(
                step["hub"], "step.hub");
        const auto velocity = debug_carcass_row64(
                step["velocity"], "step.velocity");
        const biketyre::WheelOutputs &out = model.evaluate(
                debug_carcass_span(hub), debug_carcass_span(velocity),
                wire::real(step["omega_forward_radps"],
                           "step.omega_forward_radps"),
                road, map,
                wire::real(step["dt_s"], "step.dt_s"));
        nb::dict d = debug_model_outputs_dict(out);
        d["state"] = debug_model_state_dict(model);
        return d;
    }, nb::arg("spec"), nb::arg("tier"), nb::arg("profile"),
        nb::arg("surface_map"), nb::arg("state"), nb::arg("step"),
        "Debug test surface: one PneumaticTyre.evaluate step — all "
        "WheelOutputs fields (patches included) plus the post-call "
        "carcass/patch state. step is {hub, velocity, "
        "omega_forward_radps, dt_s[, pressure]}; a non-None pressure "
        "runs set_pressure before the step.")
    ;
    }

    [[gnu::noinline]] void
    bind_pneumatic_pressure_debug(nb::module_ &m) {
        m.def("_debug_pneumatic_model_set_pressure",
              // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) named nb::args mirror the oracle's keyword order
              [](nb::handle raw_spec, nb::handle raw_tier,
                 double pressure_bar) {
        const DebugModelTier tier = debug_model_tier(raw_tier);
        biketyre::PneumaticTyreModel model(
                debug_carcass_spec(raw_spec), tier.n_rays,
                tier.half_angle_deg, tier.discretised_brush);
        model.set_pressure(pressure_bar);
        return model.tyre().pressure_bar;
    }, nb::arg("spec"), nb::arg("tier"), nb::arg("pressure_bar"),
        "Debug test surface: PneumaticTyre.set_pressure — returns the "
        "post-clamp spec pressure_bar.")
    ;
    }

    [[gnu::noinline]] void
    bind_pneumatic_reset_debug(nb::module_ &m) {
        m.def("_debug_pneumatic_model_reset",
              // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) named nb::args mirror the oracle's keyword order
              [](nb::handle raw_spec, nb::handle raw_tier,
                 nb::handle raw_state) {
        const DebugModelTier tier = debug_model_tier(raw_tier);
        biketyre::PneumaticTyreModel model(
                debug_carcass_spec(raw_spec), tier.n_rays,
                tier.half_angle_deg, tier.discretised_brush);
        debug_model_restore_state(wire::mapping(raw_state, "state"),
                                  model);
        model.reset();
        nb::dict d;
        d["outputs"] = debug_model_outputs_dict(model.last_outputs());
        d["state"] = debug_model_state_dict(model);
        return d;
    }, nb::arg("spec"), nb::arg("tier"), nb::arg("state"),
        "Debug test surface: PneumaticTyre.reset — restores the given "
        "state, clears it and returns the WheelOutputs.empty snapshot "
        "plus the cleared state.")
    ;
    }

    void bind_pneumatic_model_debug(nb::module_ &m) {
        bind_pneumatic_evaluate_debug(m);
        bind_pneumatic_pressure_debug(m);
        bind_pneumatic_reset_debug(m);
    }
} // namespace

// Binding glue builds 15-20KB frames of nanobind temporaries at import
// time; the 8KB frame guard stays armed for the per-step code.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
NB_MODULE(bike_native, m) {
    nb::object fatal_error = nb::module_::import_("mujoco").attr("FatalError");
    m.attr("FatalError") = fatal_error;
    // Nanobind retains the translator for the interpreter lifetime; keep its
    // payload alive even if module attributes are changed by Python code.
    Py_INCREF(fatal_error.ptr());
    nb::register_exception_translator(&translate_engine_failure,
                                      fatal_error.ptr());
    bind_drive_policies(m);
    auto stepper_class = nb::class_<Stepper>(m, "Stepper");
    // Isolated fatal-path test hook. It restores the model option before the
    // exception reaches Python, while Stepper retains its poisoned state.
    m.def("_engine_test_forward_failure", [](Stepper &stepper) {
        stepper.mutate([&] {
            mjModel *model = stepper.model();
            const int previous_solver = model->opt.solver;
            model->opt.solver = 99;
            try {
                stepper.forward();
            } catch (...) {
                model->opt.solver = previous_solver;
                throw;
            }
            model->opt.solver = previous_solver;
        });
    });
    m.def("_engine_test_set_const_failure", [](Stepper &stepper) {
        stepper.mutate([&] {
            mjModel *model = stepper.model();
            // body_ipos/sameframe are restored on every path, but a
            // partial mj_setConst can still leave model state behind —
            // the probe poisons anyway; stay conservative.
            stepper.note_posvel_stale();
            if (model->nbody < 2)
                throw std::invalid_argument("setConst test needs a nonworld body");
            const auto ipos = std::views::counted(model->body_ipos, 3 * model->nbody);
            const auto simple = std::views::counted(model->body_simple, model->nbody);
            const auto sameframe = std::views::counted(model->body_sameframe, model->nbody);
            const auto previous_ipos = ipos[3];
            const auto previous_simple = simple[1];
            const auto previous_sameframe = sameframe[1];
            simple[1] = 1;
            ipos[3] = previous_ipos + .123;
            try {
                engine::set_const(model, stepper.data());
            } catch (...) {
                ipos[3] = previous_ipos;
                simple[1] = previous_simple;
                sameframe[1] = previous_sameframe;
                throw;
            }
            ipos[3] = previous_ipos;
            simple[1] = previous_simple;
            sameframe[1] = previous_sameframe;
        });
    });
    m.def("_engine_test_try_status", [](Stepper &stepper, bool step) {
        return stepper.mutate([&] {
            stepper.refresh_time_callback_policy();
            mjModel *model = stepper.model();
            const int previous_solver = model->opt.solver;
            model->opt.solver = 99;
            engine::ErrorBuffer error;
            const bool succeeded = step ? stepper.try_step(error)
                                        : stepper.try_forward(error);
            model->opt.solver = previous_solver;
            return std::pair{succeeded, std::string(error.message.data())};
        });
    });
    // Debug-only oracle surface for the SignedProfile port — used by
    // tests/reference/test_native_distributed_tire.py to compare
    // biketyre::SignedProfile bitwise against terrain/profile_distance.py.
    // Not part of the runtime API; batched so the oracle's whole-array
    // validation order (finite before domain) is preserved.
    m.def("_debug_signed_distance",
          [](nb::handle raw_vertices, nb::handle raw_points) {
        using Table = nb::ndarray<const double, nb::shape<-1, 2>,
                                  nb::c_contig>;
        Table vertices;
        try {
            vertices = nb::cast<Table>(raw_vertices);
        } catch (const nb::cast_error &) {
            throw std::invalid_argument(
                "road vertices must be finite and strictly increasing in X");
        }
        const std::size_t vertex_count = vertices.shape(0);
        std::vector<std::array<double, 2>> verts(vertex_count);
        for (std::size_t i = 0; i < vertex_count; ++i)
            verts[i] = {vertices(i, 0), vertices(i, 1)};
        const biketyre::SignedProfile profile(std::move(verts));
        Table points;
        try {
            points = nb::cast<Table>(raw_points);
        } catch (const nb::cast_error &) {
            throw std::invalid_argument(
                "distance queries must be a finite (N, 2) array");
        }
        const std::size_t n = points.shape(0);
        // profile_distance.py:42-44 — the oracle validates finiteness of
        // the whole batch, then heights over the whole batch (the domain
        // gate), before the per-point loop. Keep that error order.
        for (std::size_t i = 0; i < n; ++i)
            if (!std::isfinite(points(i, 0)) || !std::isfinite(points(i, 1)))
                throw std::invalid_argument(
                    "distance queries must be a finite (N, 2) array");
        for (std::size_t i = 0; i < n; ++i)
            static_cast<void>(profile.height(points(i, 0)));
        std::vector<double> distance(n);
        std::vector<double> normal(2 * n);
        std::vector<double> point(2 * n);
        std::vector<std::int64_t> segment(n);
        std::vector<char> nonsmooth(n);
        for (std::size_t i = 0; i < n; ++i) {
            const biketyre::PointDistance row =
                profile.query({points(i, 0), points(i, 1)});
            distance[i] = row.distance;
            normal[2 * i] = row.normal[0];
            normal[2 * i + 1] = row.normal[1];
            point[2 * i] = row.point[0];
            point[2 * i + 1] = row.point[1];
            segment[i] = row.segment;
            nonsmooth[i] = static_cast<char>(row.nonsmooth);
        }
        return nb::make_tuple(as_owned(std::move(distance)),
                              as_owned_grid(std::move(normal)),
                              as_owned_grid(std::move(point)),
                              wire::owned_array<std::int64_t>(segment),
                              as_owned_bool(std::move(nonsmooth)));
    }, nb::arg("vertices"), nb::arg("points"),
        "Debug test surface: signed profile distance oracle rows "
        "(distance, normal, point, segment, nonsmooth) for (N,2) inputs.")
    ;
    // Debug-only oracle surfaces for the pneumatic geometry kernel —
    // used by tests/reference/test_native_pneumatic_geometry.py to
    // compare biketyre::RayRing / RoadProfile / intersect / patches
    // bitwise against sim/ride/tyre/geometry.py. Not part of the runtime
    // API. The half angle arrives in degrees and is converted with the
    // oracle's own expression (model.py:30,162 — deg * (np.pi/180)).
    m.def("_debug_pneumatic_ring",
          [](std::int64_t n_rays, double half_angle_deg) {
        const biketyre::RayRing ring(
            n_rays, half_angle_deg * (std::numbers::pi / 180.0));
        nb::dict d;
        d["theta_rad"] = wire::owned_array<double>(ring.theta_rad());
        d["ux"] = wire::owned_array<double>(ring.ux());
        d["uz"] = wire::owned_array<double>(ring.uz());
        d["dtheta_rad"] = ring.dtheta_rad();
        return d;
    }, nb::arg("n_rays"), nb::arg("half_angle_deg"),
        "Debug test surface: pneumatic RayRing rows "
        "(theta_rad, ux, uz, dtheta_rad) for (n_rays, half_angle_deg).")
    ;
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional args mirror the Python oracle's intersect()
    // NOLINTBEGIN(bugprone-easily-swappable-parameters) positional args mirror the Python oracle's intersect()
    m.def("_debug_pneumatic_intersect",
          [](nb::handle raw_profile, double centre_x_m, double centre_z_m,
             double radius_m, std::int64_t n_rays, double half_angle_deg) {
        // NOLINTEND(bugprone-easily-swappable-parameters)
        const wire::Dict profile{.value = wire::mapping(raw_profile,
                                                       "profile"),
                                 .path = "profile"};
        wire::exact(profile, wire::keys("x0_m", "dx_m", "z_m"));
        using Row = nb::ndarray<const double, nb::shape<-1>, nb::c_contig>;
        Row z_wire;
        try {
            z_wire = nb::cast<Row>(profile["z_m"]);
        } catch (const nb::cast_error &) {
            wire::invalid("profile.z_m",
                          "expected a contiguous float64 array");
        }
        std::vector<double> z(z_wire.shape(0));
        for (std::size_t i = 0; i < z.size(); ++i)
            z[i] = z_wire(static_cast<std::ptrdiff_t>(i));
        const biketyre::RoadProfile road(
            wire::finite_real(profile["x0_m"], "profile.x0_m"),
            wire::finite_real(profile["dx_m"], "profile.dx_m"),
            std::move(z));
        const biketyre::RayRing ring(
            n_rays, half_angle_deg * (std::numbers::pi / 180.0));
        const biketyre::RayHits hits = biketyre::intersect(
            ring, centre_x_m, centre_z_m, radius_m, road);
        nb::dict d;
        d["r_m"] = wire::owned_array<double>(hits.r_m);
        d["delta_m"] = wire::owned_array<double>(hits.delta_m);
        d["road_x_m"] = wire::owned_array<double>(hits.road_x_m);
        d["road_z_m"] = wire::owned_array<double>(hits.road_z_m);
        d["coverage_event"] = hits.coverage_event;
        d["airborne"] = hits.airborne;
        return d;
    }, nb::arg("profile"), nb::arg("centre_x_m"), nb::arg("centre_z_m"),
        nb::arg("radius_m"), nb::arg("n_rays"), nb::arg("half_angle_deg"),
        "Debug test surface: pneumatic intersect row set "
        "(r_m, delta_m, road_x_m, road_z_m, coverage_event, airborne) for "
        "a {x0_m, dx_m, z_m} profile dict.")
    ;
    m.def("_debug_pneumatic_patches",
          [](nb::handle raw_delta) {
        using Row = nb::ndarray<const double, nb::shape<-1>, nb::c_contig>;
        Row delta;
        try {
            delta = nb::cast<Row>(raw_delta);
        } catch (const nb::cast_error &) {
            wire::invalid("delta_m",
                          "expected a contiguous float64 array");
        }
        const std::span<const double> delta_m = std::views::counted(
            delta.data(), static_cast<std::ptrdiff_t>(delta.shape(0)));
        return as_owned_pairs32(biketyre::patches(delta_m));
    }, nb::arg("delta_m"),
        "Debug test surface: pneumatic patches() as an int32 (K,2) "
        "(start, stop) array for a 1-D deflection row.")
    ;
    // Debug-only oracle surfaces for the pneumatic carcass kernel —
    // used by tests/reference/test_native_pneumatic_carcass.py to run
    // identical multi-step sequences through biketyre::CarcassState /
    // evaluate_carcass and carcass.CarcassState / evaluate_carcass.
    // The spec dict is the 18-field TyreSpecs wire
    // (config.hpp:155-156); the hits dict is the six RayHits fields
    // (r_m/coverage_event/airborne ride along unused, like the oracle
    // reads them); the state dict is every CarcassState field minus
    // n_rays (the rows' shared length stands in for it).
    m.def("_debug_pneumatic_carcass",
          [](nb::handle raw_spec, std::int64_t n_rays,
             // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python surface signature
             double half_angle_deg, nb::handle raw_hits,
             nb::handle raw_state, double dt_s, double omega_radps,
             double speed_mps) {
        const nativecfg::PneumaticTyreSpec tyre =
                debug_carcass_spec(raw_spec);
        const wire::Dict hits{.value = wire::mapping(raw_hits, "hits"),
                              .path = "hits"};
        wire::exact(hits,
                    wire::keys("r_m", "delta_m", "road_x_m", "road_z_m",
                               "coverage_event", "airborne"));
        // r_m / coverage_event / airborne ride along for interface
        // fidelity — the carcass kernel never reads them, exactly like
        // evaluate_carcass in the oracle.
        const biketyre::RayHits hits_v{
                .r_m = debug_carcass_row_vec(hits, "r_m"),
                .delta_m = debug_carcass_row_vec(hits, "delta_m"),
                .road_x_m = debug_carcass_row_vec(hits, "road_x_m"),
                .road_z_m = debug_carcass_row_vec(hits, "road_z_m"),
                .coverage_event = wire::boolean(hits["coverage_event"],
                                                hits.child(
                                                        "coverage_event")),
                .airborne = wire::boolean(hits["airborne"],
                                          hits.child("airborne"))};
        biketyre::CarcassState state = debug_carcass_state(raw_state);
        const biketyre::RayRing ring(
                n_rays, half_angle_deg * (std::numbers::pi / 180.0));
        const biketyre::CarcassResult result = biketyre::evaluate_carcass(
                ring, hits_v, tyre, state, dt_s, omega_radps, speed_mps);
        nb::dict d;
        d["pressure_force_n"] =
                wire::owned_array<double>(result.pressure_force_n);
        d["elastic_force_n"] =
                wire::owned_array<double>(result.elastic_force_n);
        d["maxwell_force_n"] =
                wire::owned_array<double>(result.maxwell_force_n);
        d["hysteresis_force_n"] =
                wire::owned_array<double>(result.hysteresis_force_n);
        d["rim_force_n"] =
                wire::owned_array<double>(result.rim_force_n);
        d["total_element_force_n"] =
                wire::owned_array<double>(result.total_element_force_n);
        d["material_rate_mps"] =
                wire::owned_array<double>(result.material_rate_mps);
        d["contact_patches"] = as_owned_pairs32(result.contact_patches);
        d["contact_lengths_m"] =
                wire::owned_array<double>(result.contact_lengths_m);
        d["force_x_n"] = result.force_x_n;
        d["force_z_n"] = result.force_z_n;
        d["normal_load_n"] = result.normal_load_n;
        d["rim_energy_j"] = result.rim_energy_j;
        d["rim_strike_active"] = result.rim_strike_active;
        if (result.rim_event.has_value()) {
            nb::dict ev;
            ev["x_m"] = result.rim_event->x_m;
            ev["speed_mps"] = result.rim_event->speed_mps;
            ev["peak_load_n"] = result.rim_event->peak_load_n;
            ev["peak_rim_force_n"] =
                    result.rim_event->peak_rim_force_n;
            ev["absorbed_energy_j"] =
                    result.rim_event->absorbed_energy_j;
            d["rim_event"] = ev;
        } else {
            d["rim_event"] = nb::none();
        }
        d["state"] = debug_carcass_state_dict(state);
        return d;
    }, nb::arg("spec"), nb::arg("n_rays"), nb::arg("half_angle_deg"),
        nb::arg("hits"), nb::arg("state"), nb::arg("dt_s"),
        nb::arg("omega_radps"), nb::arg("speed_mps"),
        "Debug test surface: one evaluate_carcass step — all "
        "CarcassResult fields plus the post-call CarcassState.")
    ;
    m.def("_debug_pneumatic_advance_unloaded",
          [](nb::handle raw_state, double omega_radps, double dt_s,
             double dtheta_rad, double relaxation_s) {
        biketyre::CarcassState state = debug_carcass_state(raw_state);
        state.advance_unloaded(omega_radps, dt_s, dtheta_rad,
                               relaxation_s);
        return debug_carcass_state_dict(state);
    }, nb::arg("state"), nb::arg("omega_radps"), nb::arg("dt_s"),
        nb::arg("dtheta_rad"), nb::arg("relaxation_s"),
        "Debug test surface: CarcassState.advance_unloaded — returns "
        "the post-call state dict.")
    ;
    m.def("_debug_pneumatic_material_rate",
          // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python surface signature
          [](nb::handle raw_delta, nb::handle raw_prev,
             double omega_radps, double dt_s, double dtheta_rad) {
        const auto delta = debug_carcass_row64(raw_delta, "delta_m");
        const auto prev = debug_carcass_row64(raw_prev, "delta_prev_m");
        return wire::owned_array<double>(
                biketyre::material_deflection_rate_mps(
                        debug_carcass_span(delta),
                        debug_carcass_span(prev), omega_radps, dt_s,
                        dtheta_rad));
    }, nb::arg("delta_m"), nb::arg("delta_prev_m"),
        nb::arg("omega_radps"), nb::arg("dt_s"), nb::arg("dtheta_rad"),
        "Debug test surface: material_deflection_rate_mps row.")
    ;
    m.def("_debug_pneumatic_advect",
          [](nb::handle raw_values, double omega_radps, double dt_s,
             double dtheta_rad) {
        const auto values = debug_carcass_row64(raw_values, "values");
        return wire::owned_array<double>(biketyre::advect_material(
                debug_carcass_span(values), omega_radps, dt_s,
                dtheta_rad));
    }, nb::arg("values"), nb::arg("omega_radps"), nb::arg("dt_s"),
        nb::arg("dtheta_rad"),
        "Debug test surface: _advect_material row (unit-grid np.interp "
        "with left=right=0.0).")
    ;
    m.def("_debug_pneumatic_math",
          [](nb::handle raw_xs, nb::handle raw_ys) {
        const auto xs = debug_carcass_row64(raw_xs, "xs");
        const auto ys = debug_carcass_row64(raw_ys, "ys");
        const std::size_t nx = xs.shape(0), ny = ys.shape(0);
        const std::size_t pairs = std::min(nx, ny);
        std::vector<double> tanh_out(nx), hypot_out(pairs);
        for (std::size_t i = 0; i < nx; ++i)
            tanh_out[i] = numeric::np_tanh(xs(static_cast<std::ptrdiff_t>(i)));
        for (std::size_t i = 0; i < pairs; ++i)
            hypot_out[i] = std::hypot(xs(static_cast<std::ptrdiff_t>(i)),
                                      ys(static_cast<std::ptrdiff_t>(i)));
        nb::dict d;
        d["tanh"] = wire::owned_array<double>(tanh_out);
        d["hypot"] = wire::owned_array<double>(hypot_out);
        return d;
    }, nb::arg("xs"), nb::arg("ys"),
        "Debug test surface: np.tanh(xs) and np.hypot(xs, ys) "
        "transcendental probes — the two non-trivial lowerings the "
        "carcass kernel uses.")
    ;
    // Debug-only oracle surfaces for the pneumatic brush kernels —
    // used by tests/reference/test_native_pneumatic_brush.py to compare
    // biketyre::relax / sigma_x / tread_stiffness_n_m2 / surface_mu /
    // lumped_brush / DiscretisedBrush bitwise against
    // sim/ride/tyre/brush.py. Not part of the runtime API. Scalar
    // arguments arrive as raw doubles: each kernel's own validation
    // order is the oracle's.
    m.def("_debug_pneumatic_relax",
          // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python surface signature
          [](double kappa_prime, double v_x_mps, double v_s_mps,
             double sigma_m, double dt_s) {
        return biketyre::relax(kappa_prime, v_x_mps, v_s_mps, sigma_m,
                               dt_s);
    }, nb::arg("kappa_prime"), nb::arg("v_x_mps"), nb::arg("v_s_mps"),
        nb::arg("sigma_m"), nb::arg("dt_s"),
        "Debug test surface: brush.relax scalar update.")
    ;
    m.def("_debug_pneumatic_sigma_x",
          [](double kappa_prime) {
        return biketyre::sigma_x(kappa_prime);
    }, nb::arg("kappa_prime"),
        "Debug test surface: brush._sigma_x slip input.")
    ;
    m.def("_debug_pneumatic_surface_mu",
          [](nb::handle raw_surface, double sliding_speed_mps) {
        return biketyre::surface_mu(debug_brush_surface(raw_surface),
                                    sliding_speed_mps);
    }, nb::arg("surface"), nb::arg("sliding_speed_mps"),
        "Debug test surface: SurfaceSpec.mu Stribeck coefficient.")
    ;
    m.def("_debug_pneumatic_tread_stiffness",
          // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python surface signature
          [](double normal_load_n, double half_length_m,
             nb::handle raw_surface, nb::handle raw_tread) {
        return biketyre::tread_stiffness_n_m2(
                normal_load_n, half_length_m,
                debug_brush_surface(raw_surface),
                debug_optional_tread(raw_tread));
    }, nb::arg("normal_load_n"), nb::arg("half_length_m"),
        nb::arg("surface"), nb::arg("tread_stiffness_n_m2").none(),
        "Debug test surface: brush._tread_stiffness_n_m2 c_px "
        "resolution.")
    ;
    m.def("_debug_pneumatic_lumped_brush",
          // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python surface signature
          [](double kappa_prime, double normal_load_n,
             double half_length_m, nb::handle raw_surface,
             nb::handle raw_tread, double sliding_speed_mps) {
        return debug_brush_result_dict(biketyre::lumped_brush(
                kappa_prime, normal_load_n, half_length_m,
                debug_brush_surface(raw_surface),
                debug_optional_tread(raw_tread), sliding_speed_mps));
    }, nb::arg("kappa_prime"), nb::arg("normal_load_n"),
        nb::arg("half_length_m"), nb::arg("surface"),
        nb::arg("tread_stiffness_n_m2").none(),
        nb::arg("sliding_speed_mps"),
        "Debug test surface: brush.lumped_brush — BrushResult fields "
        "(force_n, fully_sliding, friction_coefficient).")
    ;
    m.def("_debug_pneumatic_discretised_brush",
          // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python surface signature
          [](nb::handle raw_loads, nb::handle raw_state,
             double half_length_m, double v_x_mps, double v_s_mps,
             double sigma_m, nb::handle raw_surface, double dt_s) {
        const auto loads = debug_carcass_row64(raw_loads,
                                               "normal_loads_n");
        biketyre::DiscretisedBrush brush =
                debug_brush_state(raw_state);
        const biketyre::BrushResult result = brush.step(
                debug_carcass_span(loads), half_length_m, v_x_mps,
                v_s_mps, sigma_m, debug_brush_surface(raw_surface),
                dt_s);
        nb::dict d = debug_brush_result_dict(result);
        d["state"] = debug_brush_state_dict(brush);
        return d;
    }, nb::arg("normal_loads_n"), nb::arg("state"),
        nb::arg("half_length_m"), nb::arg("v_x_mps"),
        nb::arg("v_s_mps"), nb::arg("sigma_m"), nb::arg("surface"),
        nb::arg("dt_s"),
        "Debug test surface: one DiscretisedBrush.step — BrushResult "
        "fields plus the post-call state (bristle_deflection_m, "
        "kappa_prime).")
    ;
    m.def("_debug_pneumatic_interp",
          [](double x, nb::handle raw_values, double left,
             double right) {
        const auto values = debug_carcass_row64(raw_values, "values");
        return numeric::np_interp_unit_grid(
                x, debug_carcass_span(values), left, right);
    }, nb::arg("x"), nb::arg("values"), nb::arg("left"),
        nb::arg("right"),
        "Debug test surface: np.interp(x, np.arange(n), values, "
        "left=lval, right=rval) for one query — covers the lenxp == 1 "
        "branch, NaN/+-inf queries and out-of-range sentinels.")
    ;
    bind_pneumatic_model_debug(m);
    stepper_class
            .def(nb::init<const std::string &>())
            .def(nb::init<const std::string &, nb::handle>(),
                 nb::arg("mjb_path"), nb::arg("config").none())
            .def("step", &Stepper::step)
            .def("forward", &Stepper::forward)
            .def("reset", &Stepper::reset)
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python state-array signature
            .def("set_state", [](Stepper &s, nb::handle qpos, nb::handle qvel,
                                 nb::handle act, nb::handle warmstart, nb::handle time) {
                const auto qp = wire::vector(qpos, "set_state.qpos");
                const auto qv = wire::vector(qvel, "set_state.qvel");
                const auto a = wire::vector(act, "set_state.act");
                const auto w = wire::vector(warmstart, "set_state.warmstart");
                s.set_state(qp, qv, a, w, wire::finite_real(time, "set_state.time"));
            }, nb::arg("qpos").none(), nb::arg("qvel").none(), nb::arg("act").none(),
               nb::arg("warmstart").none(), nb::arg("time").none())
            .def_prop_ro("qpos", [](Stepper &s) { return as_view(s.qpos()); },
                         nb::rv_policy::reference_internal)
            .def_prop_ro("qvel", [](Stepper &s) { return as_view(s.qvel()); },
                         nb::rv_policy::reference_internal)
            .def_prop_ro("qacc", [](Stepper &s) { return as_view(s.qacc()); },
                         nb::rv_policy::reference_internal)
            .def_prop_ro("qfrc_constraint", [](Stepper &s) {
                return as_view(s.qfrc_constraint());
            }, nb::rv_policy::reference_internal)
            // Owned snapshot, NOT a view: efc_force points into mjData's
            // constraint arena, which rewinds and reallocates its offset
            // whenever the contact set changes — a view would silently go
            // stale (or dangle) after the next forward. The capsule-owned
            // copy is frozen at access time and writeable; the capsule is
            // the owner, so automatic_reference keeps it (reference_internal
            // would reject an ndarray that already has an owner).
            .def_prop_ro("efc_force", [](Stepper &s) {
                return wire::owned_array<double>(s.efc_force());
            }, nb::rv_policy::automatic_reference)
            .def_prop_ro("ctrl", [](Stepper &s) { return as_view(s.ctrl()); },
                         nb::rv_policy::reference_internal)
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed front/rear Python signature
            .def("brake_torques", [](Stepper &s, nb::handle front, nb::handle rear) {
                const double checked_front = wire::finite_real(front, "brake_torques.front_demand");
                const double checked_rear = wire::finite_real(rear, "brake_torques.rear_demand");
                return s.brake_torques(checked_front, checked_rear);
            }, nb::arg("front_demand").none(), nb::arg("rear_demand").none())
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed front/rear Python signature
            .def("apply_brake", [](Stepper &s, nb::handle front, nb::handle rear) {
                const double checked_front = wire::finite_real(front, "apply_brake.front_demand");
                const double checked_rear = wire::finite_real(rear, "apply_brake.rear_demand");
                s.apply_brake(checked_front, checked_rear);
            }, nb::arg("front_demand").none(), nb::arg("rear_demand").none())
            .def("cruise_compute", &Stepper::cruise_compute,
                 nb::arg("rear_in_contact"), nb::arg("traction_limited") = false,
                 nb::arg("controller_grounded") = nb::none())
            .def("cruise_reset", &Stepper::cruise_reset)
            .def("cruise_set_target_speed", [](Stepper &s, nb::handle value) {
                s.cruise_set_target_speed(wire::finite_real(value, "cruise_set_target_speed.value_kmh"));
            }, nb::arg("value_kmh").none())
            .def("cruise_set_assist_compensation",
                 [](Stepper &s, nb::handle value) {
                     return s.cruise_set_assist_compensation(wire::finite_real(value, "cruise_set_assist_compensation.support_factor"));
                 }, nb::arg("support_factor").none())
            .def("cruise_state", [](const Stepper &s) {
                const CruiseState state = s.cruise_state();
                nb::dict out;
                out["target_speed_mps"] = state.target_speed_mps;
                out["integral_mps_s"] = state.integral_mps_s;
                out["torque_nm"] = state.torque_nm;
                out["engaged"] = state.engaged;
                out["gain_scale"] = state.gain_scale;
                return out;
            })
            .def("set_cruise_state", [](Stepper &s, nb::handle raw) {
                const nb::dict state = wire::mapping(raw, "cruise_state");
                // Disabled controller errors precede candidate parsing, as for
                // the other controller APIs. All reads finish before mutation.
                static_cast<void>(s.cruise_state());
                wire::exact_keys(state, wire::keys("target_speed_mps", "integral_mps_s", "torque_nm", "engaged", "gain_scale"), {}, "cruise_state");
                if (state.size() != 5)
                    throw std::invalid_argument(
                        "cruise state requires exactly target_speed_mps, "
                        "integral_mps_s, torque_nm, engaged, gain_scale");
                const auto number = [&state](const char *key) {
                    try {
                        return nativecfg::detail::req_f64(state, "cruise_state", key);
                    } catch (const nb::cast_error &) {
                        throw std::invalid_argument(std::string(key) + " must be numeric");
                    }
                };
                CruiseState candidate{.target_speed_mps = number("target_speed_mps")};
                candidate.integral_mps_s = number("integral_mps_s");
                candidate.torque_nm = number("torque_nm");
                const nb::object engaged =
                        nativecfg::detail::req(state, "cruise_state", "engaged");
                if (!nb::isinstance<nb::bool_>(engaged))
                    throw std::invalid_argument("engaged must be bool");
                candidate.engaged = nb::cast<bool>(engaged);
                candidate.gain_scale = number("gain_scale");
                s.set_cruise_state(candidate);
            }, nb::arg("state").none())
            .def("resistance_components", [](Stepper &s, nb::handle raw) {
                     const nb::dict snaps = wire::mapping(raw, "resistance_components");
                     // Named locals in Python's dict order — arg eval order is
                     // unspecified, so on malformed input the 'front' error must win.
                     wire::exact_keys(snaps, wire::keys("front", "rear"), {}, "resistance_components");
                     const OwnedTireSideInput front = side_input(snaps, "front");
                     const OwnedTireSideInput rear = side_input(snaps, "rear");
                     nb::dict out;
                     for (auto &[name, vec]:
                          s.resistance_components(front.view(), rear.view()))
                         out[name.c_str()] = as_owned(std::move(vec));
                     return out;
                 }, nb::arg("snapshots"),
                 "Compute resistance from snapshot arrays; 1-D loads/working arrays "
                 "may be converted to contiguous float64/bool for this call.")
            .def("suspension_components", [](Stepper &s) {
                nb::dict out;
                for (auto &[name, vec]: s.suspension_components())
                    out[name.c_str()] = as_owned(std::move(vec));
                return out;
            })
            .def("rider_forces_qfrc", [](Stepper &s) {
                return as_owned(s.rider_forces_qfrc());
            })
            .def("total", [](Stepper &s, nb::handle raw) {
                     const nb::dict components = wire::mapping(raw, "total");
                     // Insertion order is the contract: PyDict_Keys preserves it,
                     // and the fold inside Stepper::total mirrors
                     // ForceAccumulator.total() exactly. Nanobind may convert a 1-D
                     // ndarray's dtype/layout to contiguous float64. Copy each array
                     // into vecs while its owner is alive; reject wrong-rank or
                     // non-convertible values before the fold.
                     std::vector<std::vector<double> > vecs;
                     const nb::list keys = components.keys();
                     vecs.reserve(keys.size());
                     for (nb::handle const key: keys) {
                         const std::string name = wire::string(key, "total");
                         const nb::object value = components[key];
                         vecs.push_back(wire::vector(value, "total." + name));
                     }
                     return as_owned(s.total(vecs));
                 }, nb::arg("components"),
                 "Sum finite nv-wide 1-D arrays in dict insertion order; ndarray "
                 "dtype/layout conversion to contiguous float64 is accepted.")
            // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) fixed Python state-array signature
            .def("set_tire_state", [](Stepper &s, nb::handle raw_names, nb::handle raw_row) {
                const auto sequence = wire::sequence(raw_names, "set_tire_state.names");
                std::vector<std::string> names;
                names.reserve(sequence.size());
                for (nb::handle const value: sequence) names.push_back(wire::string(value, "set_tire_state.names"));
                const auto row_values = wire::sequence(raw_row, "set_tire_state.row");
                std::vector<double> row;
                row.reserve(row_values.size());
                // Tire state owns the named sentinel domain; the wire reader checks original types only.
                for (nb::handle const value: row_values) row.push_back(wire::real(value, "set_tire_state.row"));
                s.set_tire_state(names, row);
            }, nb::arg("names").none(), nb::arg("row").none())
            .def("tire_state", [](Stepper &s) {
                return as_owned(s.tire_state());
            })
            .def_prop_ro("tire_state_names", [](Stepper &s) {
                return s.tire_state_names();
            })
            .def("tire_qfrc", [](Stepper &s, nb::handle dt) {
                return as_owned(s.tire_qfrc(wire::finite_real(dt, "tire_qfrc.dt")));
            })
            .def("tire_snapshots", [](Stepper &s) {
                // The view throws the documented kNoTire message when no
                // tire config was given — identical to the previous
                // explicit check.
                nb::dict out;
                const auto snaps = s.tire_writer().snapshots();
                const std::array<const char *, 2> sides = {"front", "rear"};
                for (std::size_t i = 0; i < 2; ++i)
                    if (snaps[i])
                        out[sides[i]] = snapshot_dict(*snaps[i]);
                return out;
            })
            .def("tire_diagnostics", [](Stepper &s) {
                const TireWriterView view = s.tire_writer();
                nb::dict out;
                const auto diags = view.diagnostics();
                const std::array<const char *, 2> sides = {"front", "rear"};
                for (std::size_t i = 0; i < 2; ++i) {
                    if (!diags[i])
                        continue;
                    const TireDiagnostics &g = *diags[i];
                    nb::dict d;
                    if (view.is_distributed()) {
                        // distributed_tire_forces.py:204-213 — the
                        // distributed dict's own key set (no compliant-
                        // only fields like normal_speed_mps or surface).
                        d["penetration_m"] = g.penetration_m;
                        d["normal_load_n"] = g.normal_load_n;
                        d["multi_support"] = g.multi_support;
                        d["supports_multiple_contacts"] =
                                g.supports_multiple_contacts;
                        d["capability_status"] = g.capability_status;
                        d["calibration_status"] = g.calibration_status;
                        d["station_count"] = g.station_count;
                        d["active_station_count"] = g.active_station_count;
                        d["nonsmooth_station_count"] =
                                g.nonsmooth_station_count;
                        d["max_patch_count"] = g.max_patch_count;
                        d["outside_material_load_range"] =
                                g.outside_material_load_range;
                        d["outside_material_deflection_range"] =
                                g.outside_material_deflection_range;
                        d["radial_energy_j"] = g.radial_energy_j;
                        d["shear_energy_j"] = g.shear_energy_j;
                        d["brush_loss_j"] = g.brush_loss_j;
                    } else {
                        d["multi_support"] = g.multi_support;
                        d["penetration_m"] = g.penetration_m;
                        d["normal_speed_mps"] = g.normal_speed_mps;
                        d["slip_mps"] = g.slip_mps;
                        d["normal_load_n"] = g.normal_load_n;
                        d["tangent_force_n"] = g.tangent_force_n;
                        d["friction_coefficient"] = g.friction_coefficient;
                        d["surface"] = g.surface;
                        d["branch_release_loss_j"] = g.branch_release_loss_j;
                        d["brush_loss_j"] = g.brush_loss_j;
                        d["radial_energy_j"] = g.radial_energy_j;
                        d["shear_energy_j"] = g.shear_energy_j;
                        d["outside_material_load_range"] =
                                g.outside_material_load_range;
                    }
                    out[sides[i]] = d;
                }
                return out;
            })
            // _run_stand's ledger terms on the live mjData — thin forwards
            // over the writer accessors, no new physics.
            .def("tire_stored_energy", &Stepper::tire_stored_energy)
            .def("tire_brush_loss_step_j",
                 &Stepper::tire_brush_loss_step_j)
            .def("tire_radial_dissipation_power_w",
                 &Stepper::tire_radial_dissipation_power_w)
            .def("kinetic_energy", &Stepper::kinetic_energy)
            .def_prop_ro("contact_count", &Stepper::contact_count)
            .def_prop_ro("time", &Stepper::time);
    bind_drivetrain(m, stepper_class);
    bind_rider_contact_math(m);
    bind_rider_attachment(m, stepper_class);
    bind_rider_contacts(m, stepper_class);
    runtime::bind_runtime(m);
    runtime::bind_research(m);
    runtime::bind_provenance(m);
    runtime::bind_test_adapter(m);
}

NATIVE_DIAG_POP
