// writer_config.hpp — the owned typed writer-config boundary (task T07;
// migration/interface-contracts.md §1/§3).
//
// `tools.native_config.project(env)` emits {'schema': 2, '<writer>': {...}}
// carrying ONLY the fields the ported writers read. That tree reaches the
// core two ways, through ONE decode implementation:
//
//   * bike::io::Value — the T05 strict cold-path AST — via
//     nativecfg::decode(value) (Python-free, writer_config.cpp).
//   * the nanobind adapter (config.hpp) — nb::dict inputs wrap the same
//     wire in a node that delegates every read to the existing wire::
//     readers, so the binding keeps its exact type strictness
//     (numbers.Real/Integral, PyNumber_Index, ndarray bool rejection).
//
// Both feed detail::decode_root over the generic Dict<N>/node protocol
// below — one parser, one key table, one validation implementation. An
// empty config leaves every writer disabled (the Stepper(path) contract);
// every missing required key or unknown key is std::invalid_argument
// naming the full public path. Schema 1 migrates resolved fork metadata
// without reconstructing defaults. Physical invariants remain enforced
// by each typed writer constructor through the shared nativecfg::validate.
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "diag.hpp"
#include "config_types.hpp"
#include "config_validation.hpp"
#include "validation.hpp"
#include "io/value.hpp"

namespace nativecfg {

    // --- typed result ----------------------------------------------------
    // The projected config's writer sections: one std::optional per
    // writer family the wire carries (drive, cruise, suspension, brake,
    // resistance, tire, rider_forces, rider_contacts — the eight
    // section_keys entries under 'config'). An absent key means the
    // writer stays disabled.
    struct WriterConfig {
        std::optional<drivetrain::DriveConfig> drive;
        std::optional<CruiseConfig> cruise;
        std::optional<SuspensionConfig> suspension;
        std::optional<BrakeConfig> brake;
        std::optional<ResistanceConfig> resistance;
        std::optional<TireConfig> tire;
        std::optional<RiderForcesConfig> rider_forces;
        std::optional<rider::RiderContactsConfig> rider_contacts;
    };

    // The owned decode entry point: a parsed wire tree (the projected
    // config document) into the typed WriterConfig. A non-object root or
    // an unknown section key throws std::invalid_argument naming the
    // public path — the same diagnostics the nanobind adapter produces
    // for a dict input. An empty object decodes to the empty WriterConfig
    // (mirroring the adapter's empty-dict → no-writers rule).
    [[nodiscard]] WriterConfig decode(const bike::io::Value &value);

    namespace detail {
        using validation::positive;
        using validation::nonnegative;

        // Same diagnostics as wire::invalid — both spell
        // "path: reason" for std::invalid_argument. Kept twin (this header
        // is Python-free) rather than sharing the binding_readers.hpp
        // one-liner.
        [[noreturn]] inline void invalid(std::string_view path,
                                         std::string_view reason) {
            throw std::invalid_argument(std::string(path) + ": " +
                                        std::string(reason));
        }

        template <typename... Keys>
        constexpr auto keys(Keys... names) {
            return std::array<std::string_view, sizeof...(Keys)>{names...};
        }

        // ---- source-agnostic node/dict views ------------------------------
        // The decode is written once against this protocol. Each source
        // node type N — the nanobind adapter's NbNode (config.hpp) and the
        // owned-side VNode (writer_config.cpp) — supplies exactly:
        //
        //   bool is_none() const;
        //   bool is_mapping() const;
        //   bool contains(std::string_view key) const;       // mapping only
        //   N    member(std::string_view key) const;         // mapping only
        //   template <class F> void each_key(path, F) const; // mapping only
        //   bool boolean(std::string_view path) const;
        //   double real(std::string_view path) const;        // accepts inf
        //   double finite_real(std::string_view path) const;
        //   std::optional<double> optional_real(std::string_view) const;
        //   std::int64_t integer(std::string_view path) const;
        //   int integer32(std::string_view path) const;
        //   std::string string(std::string_view path) const;
        //   Seq<N> sequence(std::string_view path) const;    // gated
        //   std::size_t seq_size() const;                    // sequence only
        //   N seq_at(std::size_t) const;                     // sequence only
        //   void numeric_gate(std::string_view path) const;
        //
        // where every scalar/sequence read throws std::invalid_argument
        // "path: <reason>" with the wire:: readers' verbatim reasons.

        // Lazy element view over a node already gated to an ordered
        // sequence — seq_size/seq_at are the node's own sequence ops.
        template <class N> struct Seq {
            N list;
            [[nodiscard]] std::size_t size() const { return list.seq_size(); }
            [[nodiscard]] N at(std::size_t index) const { return list.seq_at(index); }
        };

        // Parser-local ownership of a mapping node plus its public path —
        // the generic counterpart of wire::Dict.
        template <class N> struct Dict {
            N value;
            std::string path;

            [[nodiscard]] std::string child(std::string_view key) const {
                return path + "." + std::string(key);
            }
            [[nodiscard]] bool contains(std::string_view key) const {
                return value.contains(key);
            }
            [[nodiscard]] N operator[](std::string_view key) const {
                return value.member(key);
            }
        };

        // The 'native config' name of a dict — its path with the leading
        // "config" root stripped ("config" itself and "config.<rest>" ->
        // "config"/"<rest>"; adapter-rooted dicts like "config.drive"
        // reduce to "drive").
        [[nodiscard]] inline std::string_view wire_name(
            std::string_view path) noexcept {
            if (path == "config") return path;
            // remove_prefix, not substr — substr's out_of_range arm would
            // escape this noexcept (the starts_with gate makes the prefix
            // removal statically safe).
            if (path.starts_with("config.")) path.remove_prefix(7);
            return path;
        }

        template <class N>
        [[nodiscard]] N req(const Dict<N> &d, std::string_view key) {
            if (!d.contains(key))
                throw std::invalid_argument(
                    "native config: missing key '" +
                    std::string(wire_name(d.path)) + "." + std::string(key) +
                    "'");
            return d[key];
        }

        template <class N>
        void section_keys(const Dict<N> &d, std::string_view name);

        template <class N>
        [[nodiscard]] Dict<N> req_dict(const Dict<N> &d,
                                       std::string_view key) {
            const N v = req(d, key);
            if (!v.is_mapping())
                throw std::invalid_argument(
                    "native config: '" + std::string(wire_name(d.path)) +
                    "." + std::string(key) + "' must be a dict");
            Dict<N> result{.value = v, .path = d.child(key)};
            section_keys(result, wire_name(result.path));
            return result;
        }

        // wire::field/section/exact — the "missing required key" /
        // "expected dict" family (the drive parser's flavour).
        template <class N>
        [[nodiscard]] N field(const Dict<N> &d, std::string_view key) {
            if (!d.contains(key))
                invalid(d.child(key), "missing required key");
            return d[key];
        }

        template <class N>
        [[nodiscard]] Dict<N> section(const Dict<N> &d,
                                      std::string_view key) {
            const N v = field(d, key);
            if (!v.is_mapping()) invalid(d.child(key), "expected dict");
            return {.value = v, .path = d.child(key)};
        }

        // wire::exact_keys: unknown members first (iterating the source's
        // own key order), then missing requireds — same diagnostic order.
        template <class N>
        void exact_at(const Dict<N> &d,
                      std::span<const std::string_view> required,
                      std::span<const std::string_view> optional,
                      std::string_view public_path) {
            d.value.each_key(public_path, [&](std::string_view key) {
                if (std::ranges::find(required, key) == required.end() &&
                    std::ranges::find(optional, key) == optional.end())
                    invalid(std::string(public_path) + "." +
                                std::string(key),
                            "unknown key");
            });
            for (const auto key: required)
                if (!d.contains(key))
                    invalid(std::string(public_path) + "." +
                                std::string(key),
                            "missing required key");
        }

        template <class N>
        void exact(const Dict<N> &d,
                   std::span<const std::string_view> required,
                   std::span<const std::string_view> optional = {}) {
            exact_at(d, required, optional, d.path);
        }

        template <class N>
        [[nodiscard]] std::vector<double> vector(const N &v,
                                                 std::string_view path) {
            v.numeric_gate(path);
            const Seq<N> values = v.sequence(path);
            std::vector<double> result;
            result.reserve(values.size());
            for (std::size_t i = 0; i < values.size(); ++i)
                result.push_back(values.at(i).finite_real(path));
            return result;
        }

        template <std::size_t M, class N>
        [[nodiscard]] std::array<double, M> fixed(const N &v,
                                                  std::string_view path) {
            v.numeric_gate(path);
            const Seq<N> values = v.sequence(path);
            if (values.size() != M) invalid(path, "incorrect sequence width");
            std::array<double, M> result{};
            for (std::size_t i = 0; i < M; ++i)
                result.at(i) = values.at(i).finite_real(path);
            return result;
        }

        template <class N>
        [[nodiscard]] Seq<N> sequence(const N &v, std::string_view path) {
            return v.sequence(path);
        }

        // config.hpp 'req_*' typed field readers — missing-key diagnostics
        // are the "native config: missing key" family.
        template <class N>
        [[nodiscard]] double req_f64(const Dict<N> &d,
                                     std::string_view key) {
            return req(d, key).finite_real(d.child(key));
        }
        template <class N>
        [[nodiscard]] int req_int(const Dict<N> &d, std::string_view key) {
            return req(d, key).integer32(d.child(key));
        }
        template <class N>
        [[nodiscard]] bool req_bool(const Dict<N> &d, std::string_view key) {
            return req(d, key).boolean(d.child(key));
        }
        template <class N>
        [[nodiscard]] std::string req_str(const Dict<N> &d,
                                          std::string_view key) {
            return req(d, key).string(d.child(key));
        }
        template <class N>
        [[nodiscard]] std::array<double, 3> req_vec3(const Dict<N> &d,
                                                     std::string_view key) {
            return fixed<3>(req(d, key), d.child(key));
        }
        template <class N>
        [[nodiscard]] std::array<double, 2> req_vec2(const Dict<N> &d,
                                                     std::string_view key) {
            return fixed<2>(req(d, key), d.child(key));
        }

        // drive-binding 'field' typed readers — missing-key diagnostics are
        // the "missing required key" family.
        template <class N>
        [[nodiscard]] double num(const Dict<N> &d, std::string_view key) {
            return field(d, key).finite_real(d.child(key));
        }
        template <class N>
        [[nodiscard]] bool boolean(const Dict<N> &d, std::string_view key) {
            return field(d, key).boolean(d.child(key));
        }
        template <class N>
        [[nodiscard]] int integer(const Dict<N> &d, std::string_view key) {
            return field(d, key).integer32(d.child(key));
        }
        template <class N>
        [[nodiscard]] std::string str(const Dict<N> &d,
                                      std::string_view key) {
            return field(d, key).string(d.child(key));
        }
        template <class N>
        [[nodiscard]] std::optional<double> optional_num(
            const Dict<N> &d, std::string_view key) {
            const N v = field(d, key);
            return v.is_none() ? std::nullopt
                               : std::optional(v.finite_real(d.child(key)));
        }

        // The one-section-per-branch chain keeps every branch's path
        // temporaries live under -O0 codegen, so this frame legitimately
        // exceeds the 8KB guard whose target is the per-step realtime
        // path — config projection runs once at construction, same
        // rationale as the binding glue frames in test_contracts.cpp.
        NATIVE_DIAG_PUSH
        NATIVE_DIAG_IGNORE("-Wframe-larger-than")
        template <class N>
        void section_keys(const Dict<N> &d, std::string_view name) {
            if (name == "config") {
                constexpr auto required = keys("schema");
                constexpr auto optional = keys("drive", "cruise", "suspension", "brake", "resistance", "tire", "rider_forces", "rider_contacts");
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "suspension") {
                constexpr auto required = keys("physics_mode", "joints", "air_spring", "fork_damper", "shock_damper", "coil", "end_stops");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "suspension.joints") {
                constexpr auto required = keys("fork", "shock");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "suspension.air_spring") {
                constexpr auto required = keys("stanchion_inner_diam_mm", "total_travel_mm", "pos_chamber_length_mm", "neg_chamber_length_mm", "token_volume_cm3", "max_tokens", "gamma", "atm_pressure_pa", "num_tokens", "gauge_pressure_psi");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "suspension.fork_damper") {
                constexpr auto required = keys("max_hsc", "max_lsc", "max_reb", "hsc_clicks", "lsc_clicks", "rebound_clicks", "c_lsc_min", "c_lsc_max", "c_hsc_min", "c_hsc_max", "c_reb_min", "c_reb_max", "v_knee_comp", "v_knee_reb", "total_travel_mm", "hbo_start_mm", "c_hbo_base");
                constexpr auto optional = keys("legacy_behavior");
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "suspension.shock_damper") {
                constexpr auto required = keys("max_hsc", "max_lsc", "max_reb", "hsc_clicks", "lsc_clicks", "rebound_clicks", "c_lsc_min", "c_lsc_max", "c_hsc_min", "c_hsc_max", "c_reb_min", "c_reb_max", "v_knee_comp", "v_knee_reb", "total_stroke_mm", "max_hbo", "hbo_clicks", "lockout_firm", "legacy_behavior", "hbo_start_mm", "c_hbo_min", "c_hbo_max", "lockout_preload_n", "lockout_stiffness");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "suspension.coil") {
                constexpr auto required = keys("rate_n_m", "preload_mm", "stroke_mm", "bumper_length_mm", "bumper_peak_n", "legacy_behavior");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "suspension.end_stops") {
                constexpr auto required = keys("stiffness_n_m", "damping_n_s_m");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "brake") {
                constexpr auto required = keys("torque_ceiling_nm", "taper_radps");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "cruise") {
                constexpr auto required = keys("target_speed_kmh", "kp_nm_per_mps", "ki_nm_per_mps_s", "torque_ceiling_nm");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "resistance") {
                constexpr auto required = keys("crr", "rolling_taper_rad_s", "rho_kg_m3", "cda_m2", "wind_world_mps", "point_body_m", "bodies");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "resistance.bodies") {
                constexpr auto required = keys("frame", "front_wheel", "rear_wheel");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire") {
                constexpr auto required = keys("backend", "surface_mode", "front", "rear", "significant_delta_m", "significance_fraction", "distinct_normal_deg");
                constexpr auto optional = keys("surface_map", "distributed", "pneumatic");
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.front" || name == "tire.rear") {
                // material is backend-conditional: required for compliant_2d,
                // forbidden for distributed_2d_reference and pneumatic —
                // the domain rule lives in validate(TireConfig), the key
                // table stays open.
                constexpr auto required = keys("tangent_k_n_m", "mu", "relaxation_length_m");
                constexpr auto optional = keys("material");
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.front.material" || name == "tire.rear.material") {
                constexpr auto required = keys("radial_k_n_m", "radial_c_ns_m", "pressure_pa_gauge", "provenance", "valid_load_range_n");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.distributed") {
                constexpr auto required = keys("station_count", "density", "front", "rear");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.distributed.front" || name == "tire.distributed.rear") {
                constexpr auto required = keys("density", "fitting_dataset_id", "valid_load_range_n", "valid_deflection_range_m", "calibration_status");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.pneumatic") {
                constexpr auto required = keys("tier", "n_rays", "half_angle_deg", "discretised_brush", "timestep_s", "front", "rear", "profile");
                constexpr auto optional = keys("surface");
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.pneumatic.front" || name == "tire.pneumatic.rear") {
                // The 18 TyreSpecs fields (physics/tyre.py:64-78).
                constexpr auto required = keys("name", "outer_radius_mm", "rim_radius_mm", "section_height_mm", "pressure_bar", "crr_reference", "casing_width_mm", "compressed_casing_mm", "area_factor", "carcass_stiffness_n_mm2", "contact_length_factor", "rate_stiffening", "rate_relaxation_s", "loss_factor", "hysteresis_rate_eps_mps", "relaxation_length_mm", "rim_stiffness_n_mm2", "tread_loss_crr");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.pneumatic.profile") {
                constexpr auto required = keys("x0_m", "dx_m", "z_m");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name.ends_with(".density")) {
                // tire.distributed.density and each wheel's own density law.
                constexpr auto required = keys("knots_m", "stiffness_n_m", "damping_ns_m", "provenance");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name.ends_with(".surface")) {
                constexpr auto required = keys("name", "mu_peak", "mu_slide", "slip_stiffness_per_load", "stribeck_speed_mps");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.surface_map") {
                constexpr auto required = keys("surface", "sections");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "tire.surface_map.sections") {
                constexpr auto required = keys("start_m", "end_m", "surface");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "rider_forces") {
                constexpr auto required = keys("paths");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "rider_forces.paths") {
                constexpr auto required = keys("name", "joint", "stiffness_n_m", "damping_ns_m", "preload_deflection_m", "offset_m", "unilateral");
                constexpr auto optional = keys();
                exact_at(d, required, optional, d.path);
                return;
            }
            if (name == "rider_contacts") {
                constexpr auto required = keys("arm_reach_m", "saddle_patch_half_length_m", "pedal_patch_half_length_m", "support_pad_radius_m", "support_k_n_m", "support_c_ns_m", "pedal_c_ns_m", "support_tangent_k_n_m", "support_mu", "support_length_m", "grip_k_n_m", "grip_c_ns_m", "grip_release_distance_m", "grip_capture_distance_m", "grip_capture_speed_mps", "pedal_attachment", "saddle_attachment", "grip_attachment");
                constexpr auto optional = keys("grip_pair_force_limit_n");
                exact_at(d, required, optional, d.path);
                return;
            }
        }
        NATIVE_DIAG_POP

        // Returns the parsed struct by value deliberately; -Wlarge-by-value-copy
        // exists to flag by-value *parameters*, and clang gives no param/return split.
        NATIVE_DIAG_PUSH
        NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
        template <class N>
        [[nodiscard]] DamperCore damper_core(const Dict<N> &d) {
            DamperCore c{};
            c.max_hsc = req_int(d, "max_hsc");
            c.max_lsc = req_int(d, "max_lsc");
            c.max_reb = req_int(d, "max_reb");
            // BaseDamper.__init__ clamps clicks into range; project() emits the
            // resolved attributes, so this is a no-op for well-formed input and
            // keeps hand-built dicts on the same code path.
            c.hsc_clicks = std::max(0, std::min(c.max_hsc, req_int(d, "hsc_clicks")));
            c.lsc_clicks = std::max(0, std::min(c.max_lsc, req_int(d, "lsc_clicks")));
            c.rebound_clicks =
                    std::max(0, std::min(c.max_reb, req_int(d, "rebound_clicks")));
            c.c_lsc_min = req_f64(d, "c_lsc_min");
            c.c_lsc_max = req_f64(d, "c_lsc_max");
            c.c_hsc_min = req_f64(d, "c_hsc_min");
            c.c_hsc_max = req_f64(d, "c_hsc_max");
            c.c_reb_min = req_f64(d, "c_reb_min");
            c.c_reb_max = req_f64(d, "c_reb_max");
            c.v_knee_comp = req_f64(d, "v_knee_comp");
            c.v_knee_reb = req_f64(d, "v_knee_reb");
            return c;
        }
        NATIVE_DIAG_POP

        template <class N>
        [[nodiscard]] SuspensionConfig suspension_from_dict(
            const Dict<N> &top, int schema) {
            const Dict<N> d = req_dict(top, "suspension");
            SuspensionConfig c;
            c.physics_mode = req_str(d, "physics_mode");
            if (c.physics_mode != "legacy" && c.physics_mode != "physical")
                throw std::invalid_argument(
                    "config.suspension.physics_mode must be 'legacy' or "
                    "'physical'");
            {
                const Dict<N> j = req_dict(d, "joints");
                c.fork_joint = req_str(j, "fork");
                c.shock_joint = req_str(j, "shock");
            }
            {
                const Dict<N> a = req_dict(d, "air_spring");
                c.air_spring.specs = {
                    .stanchion_inner_diam_mm = req_f64(a, "stanchion_inner_diam_mm"),
                    .total_travel_mm = req_f64(a, "total_travel_mm"),
                    .pos_chamber_length_mm = req_f64(a, "pos_chamber_length_mm"),
                    .neg_chamber_length_mm = req_f64(a, "neg_chamber_length_mm"),
                    .token_volume_cm3 = req_f64(a, "token_volume_cm3"),
                    .max_tokens = req_int(a, "max_tokens"),
                    .gamma = req_f64(a, "gamma"),
                    .atm_pressure_pa = req_f64(a, "atm_pressure_pa"),
                };
                // ForkAirSpring.__init__ clamps num_tokens into [0, max_tokens].
                c.air_spring.num_tokens = std::max(
                    0, std::min(c.air_spring.specs.max_tokens,
                                req_int(a, "num_tokens")));
                c.air_spring.gauge_pressure_psi = req_f64(a, "gauge_pressure_psi");
            }
            {
                const Dict<N> f = req_dict(d, "fork_damper");
                c.fork_damper.core = damper_core(f);
                c.fork_damper.total_travel_mm = req_f64(f, "total_travel_mm");
                c.fork_damper.hbo_start_mm = req_f64(f, "hbo_start_mm");
                c.fork_damper.c_hbo_base = req_f64(f, "c_hbo_base");
                if (schema == 1 && f.contains("legacy_behavior"))
                    invalid("config.suspension.fork_damper.legacy_behavior", "unknown schema-1 key");
                // Schema 1 retains the historical resolved branch; never reconstruct defaults.
                c.fork_damper.legacy_behavior = schema == 1
                    ? c.fork_damper.hbo_start_mm == 160.0
                    : req_bool(f, "legacy_behavior");
            }
            {
                const Dict<N> h = req_dict(d, "shock_damper");
                c.shock_damper.core = damper_core(h);
                c.shock_damper.total_stroke_mm = req_f64(h, "total_stroke_mm");
                c.shock_damper.max_hbo = req_int(h, "max_hbo");
                c.shock_damper.hbo_clicks = std::max(
                    0, std::min(c.shock_damper.max_hbo,
                                req_int(h, "hbo_clicks")));
                c.shock_damper.lockout_firm = req_bool(h, "lockout_firm");
                c.shock_damper.legacy_behavior = req_bool(h, "legacy_behavior");
                c.shock_damper.hbo_start_mm = req_f64(h, "hbo_start_mm");
                c.shock_damper.c_hbo_min = req_f64(h, "c_hbo_min");
                c.shock_damper.c_hbo_max = req_f64(h, "c_hbo_max");
                c.shock_damper.lockout_preload_n =
                        req_f64(h, "lockout_preload_n");
                c.shock_damper.lockout_stiffness =
                        req_f64(h, "lockout_stiffness");
            }
            {
                const Dict<N> k = req_dict(d, "coil");
                c.coil = {
                    .rate_n_m = req_f64(k, "rate_n_m"),
                    .preload_mm = req_f64(k, "preload_mm"),
                    .stroke_mm = req_f64(k, "stroke_mm"),
                    .bumper_length_mm = req_f64(k, "bumper_length_mm"),
                    .bumper_peak_n = req_f64(k, "bumper_peak_n"),
                    .legacy_behavior = req_bool(k, "legacy_behavior"),
                };
            }
            {
                const Dict<N> e = req_dict(d, "end_stops");
                c.end_stops = {
                    .stiffness_n_m = req_f64(e, "stiffness_n_m"),
                    .damping_n_s_m = req_f64(e, "damping_n_s_m"),
                };
            }
            return c;
        }

        template <class N>
        [[nodiscard]] BrakeConfig brake_from_dict(const Dict<N> &top) {
            const Dict<N> d = req_dict(top, "brake");
            BrakeConfig c{};
            c.torque_ceiling_nm = req_f64(d, "torque_ceiling_nm");
            c.taper_radps = req_f64(d, "taper_radps");
            return c;
        }

        template <class N>
        [[nodiscard]] ResistanceConfig resistance_from_dict(
            const Dict<N> &top) {
            const Dict<N> d = req_dict(top, "resistance");
            ResistanceConfig c;
            c.crr = req_f64(d, "crr");
            c.rolling_taper_rad_s = req_f64(d, "rolling_taper_rad_s");
            c.rho_kg_m3 = req_f64(d, "rho_kg_m3");
            c.cda_m2 = req_f64(d, "cda_m2");
            c.wind_world_mps = req_vec3(d, "wind_world_mps");
            c.point_body_m = req_vec3(d, "point_body_m");
            const Dict<N> b = req_dict(d, "bodies");
            c.frame_body = req_str(b, "frame");
            c.front_wheel_body = req_str(b, "front_wheel");
            c.rear_wheel_body = req_str(b, "rear_wheel");
            return c;
        }

        template <class N>
        [[nodiscard]] SurfaceSpec surface_spec_from_dict(
            const Dict<N> &d, std::string_view name) {
            section_keys(d, name);
            SurfaceSpec c;
            c.name = req_str(d, "name");
            c.mu_peak = req_f64(d, "mu_peak");
            c.mu_slide = req_f64(d, "mu_slide");
            c.slip_stiffness_per_load = req_f64(d, "slip_stiffness_per_load");
            c.stribeck_speed_mps = req_f64(d, "stribeck_speed_mps");
            return c;
        }

        template <class N>
        [[nodiscard]] TireParams tire_params_from_dict(
            const Dict<N> &d, std::string_view name) {
            section_keys(d, name);
            TireParams p;
            // material rides the wire iff the backend is compliant_2d — the
            // distributed backend's hinge-density law lives under
            // tire.distributed. Parse what arrives; validate(TireConfig) owns
            // the per-backend presence rule.
            if (d.contains("material")) {
                const Dict<N> m = req_dict(d, "material");
                p.material = TireMaterial{
                    .radial_k_n_m = req_f64(m, "radial_k_n_m"),
                    .radial_c_ns_m = req_f64(m, "radial_c_ns_m"),
                    .pressure_pa_gauge = req_f64(m, "pressure_pa_gauge"),
                    .provenance = req_str(m, "provenance"),
                    .valid_load_range_n = req_vec2(m, "valid_load_range_n"),
                };
            }
            p.tangent_k_n_m = req_f64(d, "tangent_k_n_m");
            p.mu = req_f64(d, "mu");
            p.relaxation_length_m = req_f64(d, "relaxation_length_m");
            return p;
        }

        template <class N>
        [[nodiscard]] HingeDensity hinge_density_from_dict(
            const Dict<N> &d, std::string_view name) {
            section_keys(d, name);
            return {
                .knots_m = vector(req(d, "knots_m"), d.child("knots_m")),
                .stiffness_n_m = vector(req(d, "stiffness_n_m"), d.child("stiffness_n_m")),
                .damping_ns_m = req_f64(d, "damping_ns_m"),
                .provenance = req_str(d, "provenance"),
            };
        }

        template <class N>
        [[nodiscard]] DistributedMaterial distributed_material_from_dict(
            const Dict<N> &d) {
            section_keys(d, wire_name(d.path));
            DistributedMaterial m;
            m.density = hinge_density_from_dict(
                req_dict(d, "density"),
                std::string(wire_name(d.path)) + ".density");
            m.fitting_dataset_id = req_str(d, "fitting_dataset_id");
            m.valid_load_range_n = req_vec2(d, "valid_load_range_n");
            m.valid_deflection_range_m = req_vec2(d, "valid_deflection_range_m");
            m.calibration_status = req_str(d, "calibration_status");
            return m;
        }

        // The 18 TyreSpecs fields under tire.pneumatic.{front,rear}
        // (physics/tyre.py:64-78). The __post_init__ domains run in
        // validate(PneumaticTyreSpec) after the struct is filled — same
        // split as the Python schema's _check_tyre_specs.
        template <class N>
        [[nodiscard]] PneumaticTyreSpec pneumatic_tyre_from_dict(
            const Dict<N> &d, std::string_view name) {
            section_keys(d, name);
            PneumaticTyreSpec t;
            t.name = req_str(d, "name");
            t.outer_radius_mm = req_f64(d, "outer_radius_mm");
            t.rim_radius_mm = req_f64(d, "rim_radius_mm");
            t.section_height_mm = req_f64(d, "section_height_mm");
            t.pressure_bar = req_f64(d, "pressure_bar");
            t.crr_reference = req_f64(d, "crr_reference");
            t.casing_width_mm = req_f64(d, "casing_width_mm");
            t.compressed_casing_mm = req_f64(d, "compressed_casing_mm");
            t.area_factor = req_f64(d, "area_factor");
            t.carcass_stiffness_n_mm2 = req_f64(d, "carcass_stiffness_n_mm2");
            t.contact_length_factor = req_f64(d, "contact_length_factor");
            t.rate_stiffening = req_f64(d, "rate_stiffening");
            t.rate_relaxation_s = req_f64(d, "rate_relaxation_s");
            t.loss_factor = req_f64(d, "loss_factor");
            t.hysteresis_rate_eps_mps = req_f64(d, "hysteresis_rate_eps_mps");
            t.relaxation_length_mm = req_f64(d, "relaxation_length_mm");
            t.rim_stiffness_n_mm2 = req_f64(d, "rim_stiffness_n_mm2");
            t.tread_loss_crr = req_f64(d, "tread_loss_crr");
            return t;
        }

        // tire.pneumatic (native/config.py::_pneumatic_section): the tier
        // name plus its resolved TIERS row, the presence-optional resolved
        // surface, both TyreSpecs sides and the road profile. The section's
        // domain rules (tier row equality, TyreSpecs and profile domains)
        // run in validate(PneumaticConfig) at the end — the parser applies
        // the same checks schema.py applies so a malformed wire fails at
        // this boundary, and the writers re-check the shared validator.
        template <class N>
        [[nodiscard]] PneumaticConfig pneumatic_from_dict(const Dict<N> &d) {
            const std::string_view name = "tire.pneumatic";
            section_keys(d, name);
            PneumaticConfig c;
            c.tier = req_str(d, "tier");
            if (c.tier != "fast" && c.tier != "detailed")
                throw std::invalid_argument(
                    "config.tire.pneumatic.tier must be 'fast' or "
                    "'detailed'");
            c.n_rays = req_int(d, "n_rays");
            c.half_angle_deg = req_f64(d, "half_angle_deg");
            c.discretised_brush = req_bool(d, "discretised_brush");
            c.timestep_s = req_f64(d, "timestep_s");
            if (d.contains("surface"))
                c.surface = surface_spec_from_dict(
                    req_dict(d, "surface"), "tire.pneumatic.surface");
            c.front = pneumatic_tyre_from_dict(
                req_dict(d, "front"), "tire.pneumatic.front");
            c.rear = pneumatic_tyre_from_dict(
                req_dict(d, "rear"), "tire.pneumatic.rear");
            {
                const Dict<N> p = req_dict(d, "profile");
                c.profile = PneumaticProfile{
                    .x0_m = req_f64(p, "x0_m"),
                    .dx_m = req_f64(p, "dx_m"),
                    .z_m = vector(req(p, "z_m"),
                                  "config.tire.pneumatic.profile.z_m"),
                };
            }
            validate(c);
            return c;
        }

        template <class N>
        [[nodiscard]] TireConfig tire_from_dict(const Dict<N> &top) {
            const Dict<N> d = req_dict(top, "tire");
            TireConfig c;
            c.backend = req_str(d, "backend");
            // All three backends are accepted at the config layer; the writer
            // boundary stays honest about which law it implements
            // (compliant_2d and distributed_2d_reference today — the pneumatic
            // writer lands with the Stepper dispatch task).
            if (c.backend != "compliant_2d" && c.backend != "distributed_2d_reference" &&
                c.backend != "pneumatic")
                throw std::invalid_argument(
                    "config.tire.backend must be 'compliant_2d', "
                    "'distributed_2d_reference' or 'pneumatic'");
            c.surface_mode = req_str(d, "surface_mode");
            if (c.surface_mode != "configured" && c.surface_mode != "track")
                throw std::invalid_argument(
                    "config.tire.surface_mode must be 'configured' or "
                    "'track'");
            c.front = tire_params_from_dict(
                req_dict(d, "front"), "tire.front");
            c.rear = tire_params_from_dict(
                req_dict(d, "rear"), "tire.rear");
            c.significant_delta_m = req_f64(d, "significant_delta_m");
            c.significance_fraction = req_f64(d, "significance_fraction");
            c.distinct_normal_deg = req_f64(d, "distinct_normal_deg");
            if (d.contains("distributed")) {
                const Dict<N> dist = req_dict(d, "distributed");
                c.distributed = DistributedConfig{
                    .station_count = req_int(dist, "station_count"),
                    .density = hinge_density_from_dict(
                        req_dict(dist, "density"),
                        "tire.distributed.density"),
                    .front = distributed_material_from_dict(
                        req_dict(dist, "front")),
                    .rear = distributed_material_from_dict(
                        req_dict(dist, "rear")),
                };
            }
            if (c.surface_mode == "track" || d.contains("surface_map")) {
                const Dict<N> sm = req_dict(d, "surface_map");
                SurfaceMap map;
                map.surface = surface_spec_from_dict(
                    req_dict(sm, "surface"), "tire.surface_map.surface");
                const N raw = req(sm, "sections");
                const auto sections = sequence(
                    raw, "config.tire.surface_map.sections");
                std::size_t section_index = 0;
                for (std::size_t i = 0; i < sections.size(); ++i) {
                    const N item = sections.at(i);
                    if (!item.is_mapping())
                        throw std::invalid_argument(
                            "native config: 'tire.surface_map.sections' entries "
                            "must be dicts");
                    const std::string section_path = "tire.surface_map.sections." + std::to_string(section_index++);
                    const Dict<N> sec{.value = item,
                                      .path = "config." + section_path};
                    section_keys(sec, "tire.surface_map.sections");
                    map.sections.push_back({
                        .start_m = req_f64(sec, "start_m"),
                        .end_m = req_f64(sec, "end_m"),
                        .surface = surface_spec_from_dict(
                            req_dict(sec, "surface"),
                            section_path + ".surface"),
                    });
                }
                c.surface_map = std::move(map);
            }
            // tire.pneumatic follows the presence rule schema.py::
            // validate_config applies (tire.distributed's mirror): required
            // iff the backend selects it, forbidden otherwise; under a
            // non-'track' surface mode the resolved surface must ship inside
            // the section. validate(TireConfig) repeats the rule for
            // programmatically built configs.
            if (c.backend == "pneumatic") {
                c.pneumatic = pneumatic_from_dict(req_dict(d, "pneumatic"));
                if (c.surface_mode != "track" && !c.pneumatic->surface)
                    invalid("config.tire.pneumatic.surface",
                            "missing required key");
            } else if (d.contains("pneumatic")) {
                invalid("config.tire.pneumatic", "unknown key");
            }
            return c;
        }

        template <class N>
        [[nodiscard]] RiderForcesConfig rider_forces_from_dict(
            const Dict<N> &top) {
            const Dict<N> d = req_dict(top, "rider_forces");
            RiderForcesConfig c;
            const N raw = req(d, "paths");
            const auto paths = sequence(raw, "config.rider_forces.paths");
            std::size_t path_index = 0;
            for (std::size_t i = 0; i < paths.size(); ++i) {
                const N item = paths.at(i);
                if (!item.is_mapping())
                    throw std::invalid_argument(
                        "native config: 'rider_forces.paths' entries must be dicts");
                const std::string path = "rider_forces.paths." + std::to_string(path_index++);
                const Dict<N> p{.value = item, .path = "config." + path};
                section_keys(p, "rider_forces.paths");
                c.paths.push_back({
                    .name = req_str(p, "name"),
                    .joint = req_str(p, "joint"),
                    .stiffness_n_m = req_f64(p, "stiffness_n_m"),
                    .damping_ns_m = req_f64(p, "damping_ns_m"),
                    .preload_deflection_m = req_f64(p, "preload_deflection_m"),
                    .offset_m = req_f64(p, "offset_m"),
                    .unilateral = req_bool(p, "unilateral"),
                });
            }
            return c;
        }

        // rider_contacts (T3b-1): the ArticulatedConfig subset
        // RiderContactApplier reads, plus the pose-resolved arm_reach_m. The
        // closed-set parses name the field on rejection like the
        // ArticulatedConfig.__post_init__ membership gates they mirror.
        template <class N>
        [[nodiscard]] rider::RiderContactsConfig rider_contacts_from_dict(
            const Dict<N> &top) {
            const Dict<N> d = req_dict(top, "rider_contacts");
            rider::RiderContactsConfig c;
            c.arm_reach_m = req_f64(d, "arm_reach_m");
            c.saddle_patch_half_length_m = req_f64(d, "saddle_patch_half_length_m");
            c.pedal_patch_half_length_m = req_f64(d, "pedal_patch_half_length_m");
            c.support_pad_radius_m = req_f64(d, "support_pad_radius_m");
            c.support_k_n_m = req_f64(d, "support_k_n_m");
            c.support_c_ns_m = req_f64(d, "support_c_ns_m");
            c.pedal_c_ns_m = req_f64(d, "pedal_c_ns_m");
            c.support_tangent_k_n_m = req_f64(d, "support_tangent_k_n_m");
            c.support_mu = req_f64(d, "support_mu");
            c.support_length_m = req_f64(d, "support_length_m");
            c.grip_k_n_m = req_f64(d, "grip_k_n_m");
            c.grip_c_ns_m = req_f64(d, "grip_c_ns_m");
            c.grip_release_distance_m = req_f64(d, "grip_release_distance_m");
            c.grip_capture_distance_m = req_f64(d, "grip_capture_distance_m");
            c.grip_capture_speed_mps = req_f64(d, "grip_capture_speed_mps");
            if (d.contains("grip_pair_force_limit_n"))
                c.grip_pair_force_limit_n =
                    d["grip_pair_force_limit_n"].optional_real(
                        "config.rider_contacts.grip_pair_force_limit_n");
            const auto pedal =
                rider::pedal_attachment(req_str(d, "pedal_attachment"));
            if (!pedal)
                invalid("config.rider_contacts.pedal_attachment",
                        "unknown attachment");
            c.pedal_attachment = *pedal;
            const auto saddle =
                rider::saddle_attachment(req_str(d, "saddle_attachment"));
            if (!saddle)
                invalid("config.rider_contacts.saddle_attachment",
                        "unknown attachment");
            c.saddle_attachment = *saddle;
            const auto grip =
                rider::grip_attachment(req_str(d, "grip_attachment"));
            if (!grip)
                invalid("config.rider_contacts.grip_attachment",
                        "unknown attachment");
            c.grip_attachment = *grip;
            return c;
        }

        // ---- drivetrain section (drive_binding.cpp's parse body, T07's
        // shared-decode host) -------------------------------------------
        template <class N>
        [[nodiscard]] drivetrain::DrivePolicyConfig decode_drive_policies(
            const Dict<N> &input,
            std::span<const std::string_view> extra_root_keys) {
            constexpr auto root = keys("gearing", "pedaling", "shifting", "assist", "battery", "hub_stiffness_nm_rad", "hub_damping_nm_s");
            exact(input, root, extra_root_keys);
            drivetrain::DrivePolicyConfig out;
            const auto g = section(input, "gearing"), p = section(input, "pedaling"), s = section(input, "shifting"),
                    a = section(input, "assist"), b = section(input, "battery");
            exact(g, keys("front_teeth","rear_teeth","chain_pitch_m"), keys());
            // The hill-hold reflex fields are ride-level PhysicalConfig members the
            // ride projection folds into pedaling (native/config.py); a bare
            // drivetrain config omits them and they default to the Python values.
            exact(p, keys("enabled","coast_above_rpm","resume_below_rpm","stop_time_s","coast_cadence_tau_s","mash_cadence_rpm","mash_torque_nm","effort_slew_nm_s"),
                    keys("rollback_brake","rollback_engage_mps","rollback_release_mps","rollback_demand"));
            exact(s, keys("enabled","cassette","target_cadence_min_rpm","target_cadence_max_rpm","shift_cooldown_s","shift_cut_duration_s","torque_factor","cadence_smoothing_tau_s","upshift_slip_limit_mps","upshift_slip_mode"), keys());
            exact(a, keys("gain","max_torque","max_power","tau","slew","engage_torque_nm","gate_min_crank_rad_s","cutoff_mps","taper_width_mps","mode"), keys("profile","torque_curve"));
            exact(b, keys("enabled","energy_j","copper_w_per_nm2","speed_w_per_rad_s2","idle_w"), keys());
            out.gearing = {
                .front_teeth = integer(g, "front_teeth"), .rear_teeth = integer(g, "rear_teeth"),
                .chain_pitch_m = positive(num(g, "chain_pitch_m"), g.child("chain_pitch_m"))
            };
            if (out.gearing.front_teeth < 3 || out.gearing.rear_teeth < 3) throw std::invalid_argument(g.path);
            out.pedaling = {
                .enabled = boolean(p, "enabled"), .coast_above_rpm = positive(num(p, "coast_above_rpm"), p.child("coast_above_rpm")),
                .resume_below_rpm = nonnegative(num(p, "resume_below_rpm"), p.child("resume_below_rpm")),
                .stop_time_s = positive(num(p, "stop_time_s"), p.child("stop_time_s")),
                .coast_cadence_tau_s = nonnegative(num(p, "coast_cadence_tau_s"), p.child("coast_cadence_tau_s")),
                .mash_cadence_rpm = nonnegative(num(p, "mash_cadence_rpm"), p.child("mash_cadence_rpm")),
                .mash_torque_nm = nonnegative(num(p, "mash_torque_nm"), p.child("mash_torque_nm")),
                .effort_slew_nm_s = nonnegative(num(p, "effort_slew_nm_s"), p.child("effort_slew_nm_s")),
                .rollback_brake = p.contains("rollback_brake") ? boolean(p, "rollback_brake") : true,
                .rollback_engage_mps = p.contains("rollback_engage_mps")
                    ? positive(num(p, "rollback_engage_mps"), p.child("rollback_engage_mps")) : .25,
                .rollback_release_mps = p.contains("rollback_release_mps")
                    ? nonnegative(num(p, "rollback_release_mps"), p.child("rollback_release_mps")) : 0.,
                .rollback_demand = p.contains("rollback_demand")
                    ? nonnegative(num(p, "rollback_demand"), p.child("rollback_demand")) : 1.
            };
            if (out.pedaling.rollback_release_mps >= out.pedaling.rollback_engage_mps ||
                out.pedaling.rollback_demand > 1.)
                throw std::invalid_argument("pedaling rollback band");
            if (out.pedaling.resume_below_rpm >= out.pedaling.coast_above_rpm || (
                    out.pedaling.mash_torque_nm > 0. && out.pedaling.mash_cadence_rpm <= 0.)) throw std::invalid_argument(
                "pedaling cadence band");
            auto &sh = out.shifting;
            sh.enabled = boolean(s, "enabled");
            {
                const auto cassette = sequence(field(s, "cassette"), s.child("cassette"));
                for (std::size_t i = 0; i < cassette.size(); ++i) {
                    const int teeth = cassette.at(i).integer32(s.child("cassette"));
                    if (teeth < 3) throw std::invalid_argument(s.child("cassette"));
                    sh.cassette.push_back(teeth);
                }
            }
            std::ranges::sort(sh.cassette);
            if (sh.cassette.empty() || std::ranges::adjacent_find(sh.cassette) != sh.cassette.end()) throw
                    std::invalid_argument(s.child("cassette"));
            sh.target_cadence_min_rpm = positive(num(s, "target_cadence_min_rpm"), s.child("target_cadence_min_rpm"));
            sh.target_cadence_max_rpm = positive(num(s, "target_cadence_max_rpm"), s.child("target_cadence_max_rpm"));
            sh.shift_cooldown_s = nonnegative(num(s, "shift_cooldown_s"), s.child("shift_cooldown_s"));
            sh.shift_cut_duration_s = nonnegative(num(s, "shift_cut_duration_s"), s.child("shift_cut_duration_s"));
            sh.torque_factor = nonnegative(num(s, "torque_factor"), s.child("torque_factor"));
            sh.cadence_smoothing_tau_s = nonnegative(num(s, "cadence_smoothing_tau_s"), s.child("cadence_smoothing_tau_s"));
            sh.upshift_slip_limit_mps = nonnegative(num(s, "upshift_slip_limit_mps"), s.child("upshift_slip_limit_mps"));
            sh.upshift_slip_mode = str(s, "upshift_slip_mode");
            if (sh.upshift_slip_mode != "legacy_signed" && sh.upshift_slip_mode != "magnitude")
                invalid(s.child("upshift_slip_mode"), "unsupported mode");
            if (sh.target_cadence_max_rpm <= sh.target_cadence_min_rpm || sh.shift_cut_duration_s > sh.shift_cooldown_s ||
                sh.torque_factor > 1.) throw std::invalid_argument(s.path);
            if (sh.enabled && std::ranges::find(sh.cassette, out.gearing.rear_teeth) == sh.cassette.end()) throw
                    std::invalid_argument(g.child("rear_teeth"));
            auto &ac = out.assist;
            ac.gain = nonnegative(num(a, "gain"), a.child("gain"));
            ac.max_torque = nonnegative(num(a, "max_torque"), a.child("max_torque"));
            ac.max_power = nonnegative(num(a, "max_power"), a.child("max_power"));
            ac.tau = positive(num(a, "tau"), a.child("tau"));
            ac.slew = positive(num(a, "slew"), a.child("slew"));
            ac.engage_torque_nm = nonnegative(num(a, "engage_torque_nm"), a.child("engage_torque_nm"));
            ac.gate_min_crank_rad_s = nonnegative(num(a, "gate_min_crank_rad_s"), a.child("gate_min_crank_rad_s"));
            ac.cutoff_mps = nonnegative(num(a, "cutoff_mps"), a.child("cutoff_mps"));
            ac.taper_width_mps = positive(num(a, "taper_width_mps"), a.child("taper_width_mps"));
            ac.mode = str(a, "mode");
            if (ac.taper_width_mps > ac.cutoff_mps) throw std::invalid_argument(a.child("taper_width_mps"));
            if (a.contains("torque_curve") && !a["torque_curve"].is_none()) {
                std::vector<std::array<double, 2> > rows;
                const auto curve = sequence(a["torque_curve"], a.child("torque_curve"));
                for (std::size_t i = 0; i < curve.size(); ++i) {
                    const auto values = sequence(curve.at(i), a.child("torque_curve"));
                    if (values.size() != 2) throw std::invalid_argument(a.child("torque_curve"));
                    const double rpm = nonnegative(values.at(0).finite_real(a.child("torque_curve")), a.child("torque_curve"));
                    const double torque = nonnegative(values.at(1).finite_real(a.child("torque_curve")), a.child("torque_curve"));
                    if (!rows.empty() && rpm <= rows.back()[0]) throw std::invalid_argument(a.child("torque_curve"));
                    rows.push_back({rpm, torque});
                }
                if (rows.size() < 2) throw std::invalid_argument(a.child("torque_curve"));
                ac.torque_curve = std::move(rows);
            }
            if (a.contains("profile") && !a["profile"].is_none()) {
                const auto profile = section(a, "profile"), gains = section(profile, "mode_gains");
                const auto emtb = sequence(field(gains, "emtb"), gains.child("emtb"));
                exact(profile, keys("mode_gains", "emtb_full_gain_at_nm"));
                exact(gains, keys("eco", "tour", "emtb", "turbo"));
                if (emtb.size() != 2) throw std::invalid_argument(gains.child("emtb"));
                ac.profile = drivetrain::MotorProfile{
                    .eco = nonnegative(num(gains, "eco"), gains.child("eco")), .tour = nonnegative(num(gains, "tour"), gains.child("tour")),
                    .emtb_low = nonnegative(emtb.at(0).finite_real(gains.child("emtb")), gains.child("emtb")),
                    .emtb_high = nonnegative(emtb.at(1).finite_real(gains.child("emtb")), gains.child("emtb")),
                    .turbo = nonnegative(num(gains, "turbo"), gains.child("turbo")),
                    .emtb_full_gain_at_nm = positive(num(profile, "emtb_full_gain_at_nm"), profile.child("emtb_full_gain_at_nm"))
                };
                if (ac.mode != "eco" && ac.mode != "tour" && ac.mode != "emtb" && ac.mode != "turbo")
                    invalid(a.child("mode"), "unsupported profiled mode");
                if (ac.profile->emtb_low > ac.profile->emtb_high)
                    invalid(gains.child("emtb"), "unordered mode gains");
            }
            out.battery = {
                .enabled = boolean(b, "enabled"), .energy_j = nonnegative(num(b, "energy_j"), b.child("energy_j")),
                .copper_w_per_nm2 = nonnegative(num(b, "copper_w_per_nm2"), b.child("copper_w_per_nm2")),
                .speed_w_per_rad_s2 = nonnegative(num(b, "speed_w_per_rad_s2"), b.child("speed_w_per_rad_s2")),
                .idle_w = nonnegative(num(b, "idle_w"), b.child("idle_w"))
            };
            out.hub_stiffness_nm_rad = positive(num(input, "hub_stiffness_nm_rad"), input.child("hub_stiffness_nm_rad"));
            out.hub_damping_nm_s = nonnegative(num(input, "hub_damping_nm_s"), input.child("hub_damping_nm_s"));
            return out;
        }

        // config.drive itself: the policy subtree plus the drive-only
        // scalars — the cross-field exclusion rules (rotor/clutch
        // exclusive, elastic_chain rejects both and shifting, passive
        // drive modes reject rotor/clutch) stay in this one decode so the
        // nb adapter and the Value boundary share them verbatim.
        template <class N>
        [[nodiscard]] drivetrain::DriveConfig decode_drive(const Dict<N> &d) {
            drivetrain::DriveConfig c;
            constexpr auto extra_keys = keys("drive_mode","transmission_model","human_torque_nm","torque_ripple","crank_phase_rad","chain_k_n_m","chain_c_ns_m","bearing_c_nms_rad","rotor_inertia_kgm2","motor_clutch","brake_ceiling_nm");
            c.policies = decode_drive_policies(d, extra_keys);
            c.drive_mode = str(d, "drive_mode");
            c.transmission_model = str(d, "transmission_model");
            if (c.drive_mode != "crank_effort" && c.drive_mode != "articulated_effort" &&
                c.drive_mode != "coast" && c.drive_mode != "ideal_speed_control")
                invalid(d.child("drive_mode"), "unsupported mode");
            if (c.transmission_model != "elastic_chain" &&
                c.transmission_model != "ideal_mid_drive" &&
                c.transmission_model != "geometric_ideal_mid_drive")
                invalid(d.child("transmission_model"), "unsupported model");
            c.human_torque_nm = nonnegative(num(d, "human_torque_nm"), "human_torque_nm");
            c.torque_ripple = nonnegative(num(d, "torque_ripple"), "torque_ripple");
            if (c.torque_ripple >= 1.)
                throw std::invalid_argument("torque_ripple");
            c.crank_phase_rad = num(d, "crank_phase_rad");
            c.chain_k_n_m = positive(num(d, "chain_k_n_m"), "chain_k_n_m");
            c.chain_c_ns_m = nonnegative(num(d, "chain_c_ns_m"), "chain_c_ns_m");
            c.bearing_c_nms_rad = nonnegative(num(d, "bearing_c_nms_rad"), "bearing_c_nms_rad");
            c.rotor_inertia_kgm2 =
                    nonnegative(num(d, "rotor_inertia_kgm2"), "rotor_inertia_kgm2");
            c.brake_ceiling_nm =
                    nonnegative(num(d, "brake_ceiling_nm"), "brake_ceiling_nm");
            c.motor_clutch = boolean(d, "motor_clutch");
            if (c.motor_clutch && c.rotor_inertia_kgm2 > 0.)
                throw std::invalid_argument("rotor and clutch are exclusive");
            if (c.transmission_model == "elastic_chain" &&
                (c.motor_clutch || c.rotor_inertia_kgm2 > 0. || c.policies.shifting.enabled))
                throw std::invalid_argument(
                    "rotor/clutch/shifting requires ideal transmission");
            if ((c.drive_mode == "coast" || c.drive_mode == "ideal_speed_control") &&
                (c.motor_clutch || c.rotor_inertia_kgm2 > 0.))
                throw std::invalid_argument("passive drive cannot use rotor/clutch");
            return c;
        }

        // Whole-config decode: a non-empty wire must name the schema and
        // may carry each writer's section. Callers gate the empty mapping
        // (decode() for the Value side, the nb adapter for the dict side).
        template <class N>
        [[nodiscard]] WriterConfig decode_root(const Dict<N> &d) {
            section_keys(d, "config");
            const int schema = req_int(d, "schema");
            if (schema != 1 && schema != 2)
                throw std::invalid_argument(
                    "config.schema: unsupported schema " + std::to_string(schema));
            WriterConfig c;
            if (d.contains("drive"))
                c.drive = decode_drive(req_dict(d, "drive"));
            if (d.contains("cruise")) {
                const Dict<N> sec = req_dict(d, "cruise");
                c.cruise = CruiseConfig{
                    .target_speed_kmh = req_f64(sec, "target_speed_kmh"),
                    .kp_nm_per_mps = req_f64(sec, "kp_nm_per_mps"),
                    .ki_nm_per_mps_s = req_f64(sec, "ki_nm_per_mps_s"),
                    .torque_ceiling_nm = req_f64(sec, "torque_ceiling_nm")
                };
            }
            if (d.contains("suspension"))
                c.suspension = suspension_from_dict(d, schema);
            if (d.contains("brake"))
                c.brake = brake_from_dict(d);
            if (d.contains("resistance"))
                c.resistance = resistance_from_dict(d);
            if (d.contains("tire"))
                c.tire = tire_from_dict(d);
            if (d.contains("rider_forces"))
                c.rider_forces = rider_forces_from_dict(d);
            if (d.contains("rider_contacts"))
                c.rider_contacts = rider_contacts_from_dict(d);
            return c;
        }
    } // namespace detail
} // namespace nativecfg
