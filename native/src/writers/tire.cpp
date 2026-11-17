// writers/tire.cpp — expression-for-expression port of tire_forces.py's
// compute path plus physics/tire.py's material laws and the contact_state.py
// snapshot/patch validation. Comments cite file:line at porting time.
//
// The same-libcall argument from writers/resistance.cpp applies everywhere
// a `@`/`np.dot`/`np.linalg.norm` appears: numpy resolves those to Apple
// Accelerate's ddot/dgemv, and this TU invokes the same CBLAS symbols.
// `np.einsum('ij,ij->i')` is NOT a BLAS call — it is numpy's seeded
// 0.0+p0+p1 accumulation — replicated literally. Python `sum()` gets the
// Neumaier compensation; snapshot `round(time/dt)` is
// interval_clock::interval_id (half-even without fenv, exact int64
// bound); `min`/`max` are std::min/std::max (identical
// `b<a?b:a` / `a<b?b:a` semantics, NaN included).
#include "tire.hpp"
#include "tire_common.hpp"
#include "../config_validation.hpp"

#include "../contact/laws.hpp"
#include "../diag.hpp"
#include "../engaged.hpp"
#include "../engine_call.hpp"
#include "../interval_clock.hpp"
#include "../model_access.hpp"
#include "../numeric_sincos.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_compliant.hpp"
#include "pyfloat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <limits>
#include <new>
#include <numbers>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "../cblas_abi.hpp"

namespace {
    using Vec3 = std::array<double, model_access::kXYZ>;

    // physical_mapping.py:6-11 — resolve_id. Site contract: the wheel
    // contact geoms take the plain name check (id >= 0) — the shared
    // resolver's physical gate stays off for GEOM kinds.
    int resolve_id(const mjModel *m, mjtObj kind, const char *name) {
        return model_access::resolve_id(m, kind, name);
    }

    // physical_mapping.py:24-37 — point_jacobian_into (jacp only). The
    // shared checked boundary keeps this site's recorded contract: the
    // split body gate (range via require_id, then nonzero → the
    // physical-body message), finite point, exact 3*nv destination.
    void point_jacobian_into(const mjModel *m, const mjData *d, int body_id,
                             const Vec3 &point, std::span<mjtNum> jp) {
        model_access::point_jacobian_into(
            m, d, body_id, point, jp,
            model_access::JacobianBody::physical, "point Jacobian body");
    }

    // np.linalg.norm(v) on a (3,) — sqrt(ddot(v,v)), the same-libcall form.
    // Distinct from the CPython vector_norm (numeric_norm.hpp): this site's
    // oracle is numpy's BLAS path, so it stays on ddot by design.
    double norm3(const Vec3 &v) {
        return std::sqrt(blas::ddot(model_access::kXYZ, v.data(), 1,
                                    v.data(), 1));
    }

    Vec3 sub(const Vec3 &a, const Vec3 &b) {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    double dot3(const Vec3 &a, const Vec3 &b) {
        return blas::ddot(model_access::kXYZ, a.data(), 1, b.data(), 1);
    }

    // CPython 3.13 builtin sum() on float items: Neumaier compensated
    // summation — see writers/resistance.cpp for the derivation.
    // `sum(iterable)` with no
    // explicit start seeds int 0 (compares equal to 0.0 downstream). The
    // accumulator form exists so loops that produced a scratch vector just
    // to reduce it can feed the same per-element sequence inline — the
    // operation order, and therefore the rounding, is unchanged. Every
    // ported site in this TU feeds float patch fields or the 1.0 literal,
    // so no PyLong-addend naive steps are reachable.
    struct NeumaierSum {
        void add(double x) {
            const double t = f_result + x;
            if (std::isfinite(t)) {
                if (std::abs(f_result) >= std::abs(x))
                    c_sum += (f_result - t) + x;
                else
                    c_sum += (x - t) + f_result;
            }
            f_result = t;
        }

        [[nodiscard]] double total() const { return f_result + c_sum; }

        double f_result = 0.0;
        double c_sum = 0.0;
    };

    // SurfaceSpec.__post_init__ (surface.py:54-69) — the checks project()'s
    // input already passed; kept so a hand-built dict fails the same way.
    void validate_surface(const nativecfg::SurfaceSpec &s) {
        if (!(s.mu_peak > 0.0) || !(s.mu_slide > 0.0) ||
            !(s.slip_stiffness_per_load > 0.0) || !(s.stribeck_speed_mps > 0.0))
            throw std::invalid_argument("surface coefficients must be positive");
        if (s.mu_slide > s.mu_peak)
            throw std::invalid_argument(
                "surface mu_slide exceeds mu_peak");
    }

    // ContactPatch.__post_init__ (contact_state.py:32-59) — the fields the
    // tire writer supplies: source 'terrain', no couple/native force.
    // Returned by value deliberately; -Wlarge-by-value-copy cannot split
    // by-value parameters (its useful half) from return values.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
    TirePatch make_patch(Vec3 point, Vec3 normal, double load, double force,
                         double slip) {
        for (const double v: point)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "contact point must be a finite 3D vector");
        for (const double v: normal)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "contact normal must be a finite 3D vector");
        if (std::abs(norm3(normal) - 1.0) > 1e-8 ||
            std::abs(normal[1]) > 1e-8)
            throw std::invalid_argument(
                "expected a unit normal in the X-Z plane");
        if (!(std::isfinite(load) && std::isfinite(force) &&
              std::isfinite(slip)) || load < 0.0)
            throw std::invalid_argument(
                "invalid contact load, force or slip");
        return {
            .point_m = point, .normal = normal, .normal_load_n = load, .tangent_force_n = force, .slip_mps = slip,
            .working_surface = true,
            .native_world_force_n = std::nullopt,
            .couple_world_nm = {}
        };
    }

    NATIVE_DIAG_POP

    // WheelContactSnapshot.effective_radius_m (contact_state.py:172-181).
    // R1: terms/weights were two heap vectors reduced by py_sum — fused
    // into the same sequential Neumaier accumulators (identical operation
    // order and rounding, no per-tick scratch).
    double effective_radius(const std::vector<TirePatch> &patches,
                            const Vec3 &axis) {
        if (patches.empty())
            return 0.0;
        const bool any_loaded = std::ranges::any_of(
            patches, [](const TirePatch &p) { return p.normal_load_n > 0.0; });
        NeumaierSum terms, weights;
        for (const TirePatch &p: patches) {
            const double w = any_loaded ? p.normal_load_n : 1.0;
            // float(np.dot(p.point_m - axis, p.normal)) → ddot
            const double drop = -dot3(sub(p.point_m, axis), p.normal);
            terms.add(w * drop);
            weights.add(w);
        }
        return std::max(0.0, terms.total() / weights.total());
    }
} // namespace

// Caller preconditions (documented at the raw boundary): `d` must be an
// mjData created from `m` — dimension equality is a width check, never a
// pairing proof — and a single Stepper context owns the writer for the
// model's lifetime so `m_`/`d` outlive every call.
TireWriter::TireWriter(const mjModel *m, const mjData *d,
                       nativecfg::TireConfig config)
    : m_(m), cfg_(std::move(config)), nv_(static_cast<int>(m->nv)) {
    if (m->nv < 0 || m->nv > std::numeric_limits<int>::max())
        throw std::invalid_argument(
            "tire writer needs a model width in int range");
    nativecfg::validate(cfg_);
    // This writer is the compliant_2d port. The Stepper dispatch routes
    // 'distributed_2d_reference' to DistributedTireWriter and
    // 'pneumatic' to PneumaticTireWriter — a direct TireWriter
    // construction with either is a dispatch bug; reject the backend
    // by name here instead of dereferencing the absent TireSpec
    // material below.
    if (cfg_.backend != "compliant_2d")
        throw std::runtime_error(
            "TireWriter implements only the compliant_2d backend; "
            "'distributed_2d_reference' is served by "
            "DistributedTireWriter and 'pneumatic' by "
            "PneumaticTireWriter through the Stepper dispatch");
    // Dataclass __post_init__ validation happened when Python built the
    // config — before TireForceApplier.__init__ — so these checks precede
    // the ctor body like they do in the Python flow.
    // TireSpec.__post_init__ (tire.py:52-69) on both sides.
    for (const nativecfg::TireParams *p: {&cfg_.front, &cfg_.rear}) {
        // Engaged by the backend gate + validate() above — restated so the
        // optional access stays checked (an invariant break is a logic
        // error, not bad caller input).
        if (!p->material)
            throw std::logic_error(
                "compliant_2d tire params carry no material");
        const nativecfg::TireMaterial &mat = *p->material;
        if (!std::isfinite(mat.radial_k_n_m) ||
            !std::isfinite(mat.loss_factor) ||
            !std::isfinite(mat.hysteresis_rate_eps_mps) ||
            !std::isfinite(mat.pressure_pa_gauge))
            throw std::invalid_argument(
                "tire material scalars must be finite");
        if (!mat.table && mat.radial_k_n_m <= 0.0)
            throw std::invalid_argument("radial_k_n_m must be positive");
        // Decisions 22/39 (2026-11-10): the radial loss is the
        // rate-independent hysteresis η·kδ·tanh(δ̇/ε); a viscous
        // radial_c_ns_m term no longer exists.
        if (mat.loss_factor < 0.0 || mat.loss_factor >= 1.0 ||
            mat.pressure_pa_gauge < 0.0)
            throw std::invalid_argument(
                "loss factor must lie in [0, 1) and gauge pressure must "
                "be nonnegative");
        if (mat.hysteresis_rate_eps_mps <= 0.0)
            throw std::invalid_argument(
                "hysteresis_rate_eps_mps must be positive");
        if (mat.provenance.find_first_not_of(" \t\n\v\f\r") ==
            std::string::npos)
            throw std::invalid_argument(
                "provenance must be a nonempty description, e.g. synthetic");
        const auto [lo, hi] = mat.valid_load_range_n;
        if (!std::isfinite(lo) || !std::isfinite(hi) ||
            !(0.0 <= lo && lo < hi))
            throw std::invalid_argument(
                "valid load bounds must satisfy 0 <= minimum < maximum");
        if (mat.table) radial_tables_[p == &cfg_.front ? 0 : 1].emplace(*mat.table);
        // TireParameters.__post_init__ (physical_config.py:29-34).
        if (!std::isfinite(p->tangent_k_n_m) || p->tangent_k_n_m <= 0.0)
            throw std::invalid_argument("invalid tangent_k_n_m");
        if (!std::isfinite(p->relaxation_length_m) ||
            p->relaxation_length_m <= 0.0)
            throw std::invalid_argument("invalid relaxation_length_m");
        if (!std::isfinite(p->mu) || p->mu < 0.0)
            throw std::invalid_argument("invalid tire friction");
    }
    // TireBackendConfig.__post_init__ threshold checks (physical_config.py:
    // 59-63) — ProfileQuery repeats them; mirroring both like Python does.
    if (!std::isfinite(cfg_.significant_delta_m) ||
        cfg_.significant_delta_m < 0.0 ||
        !std::isfinite(cfg_.significance_fraction) ||
        cfg_.significance_fraction < 0.0 ||
        !std::isfinite(cfg_.distinct_normal_deg) ||
        cfg_.distinct_normal_deg <= 0.0)
        throw std::invalid_argument("invalid multi-support thresholds");
    if (cfg_.significance_fraction > 1.0 ||
        cfg_.distinct_normal_deg >= 180.0)
        throw std::invalid_argument("invalid multi-support thresholds");
    // tire_forces.py:68-70 — the surface-map gate runs before any model
    // lookups, so a track-mode config without a map fails here first.
    if (cfg_.surface_mode == "track") {
        if (!cfg_.surface_map)
            throw std::invalid_argument(
                "track material mode requires an explicit SurfaceMap");
        // SurfaceMap.__init__ (surface.py:168-182): sorted by start_m,
        // non-overlapping; plus the section/spec __post_init__ checks.
        surface_map_ = *cfg_.surface_map;
        validate_surface(surface_map_->surface);
        std::ranges::sort(surface_map_->sections, {},
                          [](const auto &s) { return s.start_m; });
        for (const auto &sec: surface_map_->sections) {
            if (!std::isfinite(sec.start_m) || !std::isfinite(sec.end_m))
                throw std::invalid_argument(
                    "surface interval stations must be finite real numbers");
            if (!(0.0 <= sec.start_m && sec.start_m < sec.end_m))
                throw std::invalid_argument(
                    "surface interval must satisfy 0 <= start < end");
            validate_surface(sec.surface);
        }
        for (std::size_t i = 1; i < surface_map_->sections.size(); ++i)
            if (surface_map_->sections[i].start_m <
                surface_map_->sections[i - 1].end_m)
                throw std::invalid_argument(
                    "material intervals must not overlap");
    }
    // tire_forces.py:71-75 — ProfileQuery over the compiled raster.
    const std::vector<std::array<double, 2> > vertices =
            biketyre::compiled_profile_vertices(m, d, "terrain");
    {
        std::vector<double> px, pz;
        px.reserve(vertices.size());
        pz.reserve(vertices.size());
        for (const auto &v: vertices) {
            px.push_back(v[0]);
            pz.push_back(v[1]);
        }
        profile_.emplace(std::move(px), std::move(pz),
                         cfg_.significant_delta_m,
                         cfg_.significance_fraction,
                         cfg_.distinct_normal_deg);
    }
    // tire_forces.py:76-79 — geoms/bodies/radii for ('front','rear').
    geoms_ = {
        resolve_id(m, mjOBJ_GEOM, "geom_front_contact"),
        resolve_id(m, mjOBJ_GEOM, "geom_rear_contact")
    };
    const std::span<const int> body_of =
            model_access::readonly_buffer(m->geom_bodyid, m->ngeom);
    const std::span<const mjtNum> gsize = model_access::readonly_buffer(
        m->geom_size, model_access::kXYZ * m->ngeom);
    for (std::size_t i = 0; i < 2; ++i) {
        model_access::require_id(geoms_[i], m->ngeom, "tire contact geom");
        bodies_[i] = body_of[static_cast<std::size_t>(geoms_[i])];
        radii_[i] = gsize[model_access::kXYZ *
                          static_cast<std::size_t>(geoms_[i])];
    }
    jac_contact_.resize(static_cast<std::size_t>(model_access::kXYZ) *
                        static_cast<std::size_t>(nv_));
    jac_center_.resize(static_cast<std::size_t>(model_access::kXYZ) *
                       static_cast<std::size_t>(nv_));
    // R1: the former per-call qfrc/tmp vectors and the snapshot staging
    // slots are construction-sized members — compute fills this capacity.
    tmp_.resize(static_cast<std::size_t>(nv_));
    qfrc_out_.resize(static_cast<std::size_t>(nv_));
    for (TireSnapshot &stage: snapshot_stage_)
        stage.patches.reserve(kMaxSnapshotPatches);
    // The committed slots carry the same construction sizing: an
    // optional-engagement commit would construct the patch vector and
    // surface string inside the marked compute (RTSan flagged exactly
    // that on the first call). `surface` capacity covers every configured
    // material name plus the 'configured' fallback — the longest name a
    // diagnostics commit can ever assign.
    std::size_t surface_name_capacity =
            std::string_view("configured").size();
    if (surface_map_) {
        surface_name_capacity = std::max(
                surface_name_capacity, surface_map_->surface.name.size());
        for (const nativecfg::SurfaceSection &sec: surface_map_->sections)
            surface_name_capacity =
                    std::max(surface_name_capacity, sec.surface.name.size());
    }
    for (std::size_t s = 0; s < 2; ++s) {
        snapshots_[s].patches.reserve(kMaxSnapshotPatches);
        diagnostics_[s].surface.reserve(surface_name_capacity);
        diagnostics_stage_[s].surface.reserve(surface_name_capacity);
    }
    // tire_forces.py:84-88 — every geom on each wheel body must be
    // collision-disabled for the compliant_2d backend.
    const std::span<const int> contype =
            model_access::readonly_buffer(m->geom_contype, m->ngeom);
    const std::span<const int> conaffinity =
            model_access::readonly_buffer(m->geom_conaffinity, m->ngeom);
    for (const int body: bodies_)
        for (int g = 0; g < m->ngeom; ++g)
            if (body_of[static_cast<std::size_t>(g)] == body &&
                (contype[static_cast<std::size_t>(g)] ||
                 conaffinity[static_cast<std::size_t>(g)]))
                throw std::invalid_argument(
                    "native wheel collisions must be disabled for "
                    "compliant_2d");
    auto model = std::make_shared<bike::physics_core::TyreModel<double>>();
    model->backend = bike::physics_core::TyreBackendKind::compliant_2d;
    model->nq = static_cast<bike::physics_core::i32>(m->nq);
    model->nv = nv_;
    model->nbody = static_cast<bike::physics_core::i32>(m->nbody);
    model->njnt = static_cast<bike::physics_core::i32>(m->njnt);
    model->distinct_normal_cosine = numeric::np_cos(
        cfg_.distinct_normal_deg * (std::numbers::pi / 180.0));
    for (std::size_t s = 0; s < 2; ++s) {
        const nativecfg::TireParams &pcfg = s == 0 ? cfg_.front : cfg_.rear;
        const nativecfg::TireMaterial &mat = engaged(pcfg.material);
        auto &w = std::span{model->wheels}[s];
        w.geom_id = geoms_[s];
        w.radius_m = radii_[s];
        w.tangent_k_n_m = pcfg.tangent_k_n_m;
        w.tire_mu = pcfg.mu;
        w.relaxation_length_m = pcfg.relaxation_length_m;
        w.compliant.radial_k_n_m = mat.radial_k_n_m;
        w.compliant.loss_factor = mat.loss_factor;
        w.compliant.hysteresis_rate_eps_mps = mat.hysteresis_rate_eps_mps;
        w.compliant.valid_load_min_n = mat.valid_load_range_n[0];
        w.compliant.valid_load_max_n = mat.valid_load_range_n[1];
        w.compliant.has_tabulated_radial = mat.table.has_value() ? 1 : 0;
    }
    core_surface_map_complete_ = surface_map_
        ? tirecommon::fill_surface_map_view(&*surface_map_, model->surface_map)
        : true;
    core_model_ = std::move(model);
    // reset() (tire_forces.py:91-98) — the members' initializers already
    // hold these values; no snapshots/diagnostics until the first compute.
}

void TireWriter::reset() {
    states_ = {};
    // Copy-assign from a const empty (not fill(std::nullopt)/move-assign):
    // the empty source keeps every destination's patch vector and surface
    // string capacity — the committed slots stay construction-sized for
    // the marked commit, and `committed_` carries the observable
    // "empty until the first advancing compute" state.
    static const TireSnapshot empty_snapshot;
    static const TireDiagnostics empty_diagnostics{};
    for (TireSnapshot &snap: snapshots_)
        snap = empty_snapshot;
    for (TireDiagnostics &diag: diagnostics_)
        diag = empty_diagnostics;
    committed_ = false;
    elastic_energy_j_ = 0.;
    brush_loss_step_j_ = 0.;
    radial_dissipation_power_w_ = 0.;
    last_time_s_.reset();
}

const std::vector<std::string> &TireWriter::state_names() {
    // flatten_row(asdict(_BrushState)) schema per side: vector fields emit
    // component columns only (the leaf 'front.tangent' form decodes but is
    // never emitted).
    static const std::vector<std::string> names = [] {
        std::vector<std::string> v;
        v.reserve(22);
        for (const char *side: {"front", "rear"})
            for (const char *f:
                 {
                     "xi", "tangent.0", "tangent.1", "tangent.2",
                     "point.0", "point.1", "point.2", "segment",
                     "center.0", "center.1", "center.2"
                 })
                v.push_back(std::string(side) + "." + f);
        return v;
    }();
    return names;
}

void TireWriter::set_state(std::span<const std::string> names,
                           std::span<const double> row) {
    prepare_state(names, row);
    commit_prepared_state();
}

void TireWriter::prepare_state(std::span<const std::string> names,
                              std::span<const double> row) {
    if (names.size() != row.size())
        throw std::invalid_argument(
            "set_tire_state: names and row must have equal length");
    // Canonical-schema gate: every artifact row carries the full canonical
    // column set (component columns are NaN when the field is unset), so a
    // missing canonical column is reported before any unexpected one.
    for (const std::string &want: state_names())
        if (!std::ranges::contains(names, want))
            throw std::invalid_argument(
                "set_tire_state: missing tire-state column '" + want + "'");
    // Field accumulators: component presence is tracked separately from a
    // leaf column (the leaf holds None/NaN whenever the row encodes an
    // unset vector).
    struct VecAcc {
        std::array<bool, model_access::kXYZ> comp_seen = {false, false,
                                                          false};
        std::array<double, model_access::kXYZ> comp = {0.0, 0.0, 0.0};
    };
    std::array<BrushState, 2> next;
    std::array<bool, 2> xi_seen = {false, false};
    std::array<bool, 2> seg_seen = {false, false};
    std::array<VecAcc, 2> tangent, point, center;
    for (std::size_t i = 0; i < names.size(); ++i) {
        const std::string &name = names[i];
        const double v = row[i];
        // '<side>.<field>[.<idx>]' — split on '.', at most 3 parts.
        const std::size_t d1 = name.find('.');
        const std::size_t d2 =
                d1 == std::string::npos
                    ? std::string::npos
                    : name.find('.', d1 + 1);
        const std::string side_s = name.substr(0, d1);
        const std::string field =
                d1 == std::string::npos
                    ? ""
                    : name.substr(d1 + 1,
                                  d2 == std::string::npos
                                      ? std::string::npos
                                      : d2 - d1 - 1);
        const std::string sub =
                d2 == std::string::npos ? "" : name.substr(d2 + 1);
        const bool is_front = side_s == "front";
        if (!is_front && side_s != "rear")
            throw std::invalid_argument("set_tire_state: unknown column '" +
                                        name + "'");
        const std::size_t s = is_front ? 0 : 1;
        if (d2 != std::string::npos &&
            (sub.size() != 1 || sub[0] < '0' || sub[0] > '2' ||
             field == "xi" || field == "segment"))
            throw std::invalid_argument(
                "set_tire_state: malformed column '" + name + "'");
        if (field == "xi") {
            if (xi_seen[s])
                throw std::invalid_argument(
                    "set_tire_state: duplicate column '" + name + "'");
            if (!std::isfinite(v))
                throw std::invalid_argument(
                    "set_tire_state: '" + name + "' must be finite");
            xi_seen[s] = true;
            next[s].xi = v;
        } else if (field == "segment") {
            if (seg_seen[s])
                throw std::invalid_argument(
                    "set_tire_state: duplicate column '" + name + "'");
            seg_seen[s] = true;
            if (!std::isnan(v) &&
                (!std::isfinite(v) || std::trunc(v) != v ||
                 v < static_cast<double>(std::numeric_limits<int>::min()) ||
                 v > static_cast<double>(std::numeric_limits<int>::max())))
                throw std::invalid_argument(
                    "set_tire_state: '" + name +
                    "' must be NaN or a finite integer in int range");
            // Cast only after validation; next is committed after the whole
            // row validates, so rejection preserves states_ and the clock.
            next[s].segment = std::isnan(v)
                                  ? std::nullopt
                                  : std::optional<int>(static_cast<int>(v));
        } else {
            VecAcc *acc = nullptr;
            if (field == "tangent") {
                acc = &tangent[s];
            } else if (field == "point") {
                acc = &point[s];
            } else if (field == "center") {
                acc = &center[s];
            } else {
                throw std::invalid_argument(
                    "set_tire_state: unknown column '" + name + "'");
            }
            if (sub.empty()) {
                // Leaf column: the artifact only ever stores None here.
                if (!std::isnan(v))
                    throw std::invalid_argument(
                        "set_tire_state: non-scalar leaf value in '" +
                        name + "'");
                continue;
            }
            const int idx = sub[0] - '0';
            if (acc->comp_seen[static_cast<std::size_t>(idx)])
                throw std::invalid_argument(
                    "set_tire_state: duplicate column '" + name + "'");
            acc->comp_seen[static_cast<std::size_t>(idx)] = true;
            acc->comp[static_cast<std::size_t>(idx)] = v;
        }
    }
    const auto fold = [](VecAcc &a, std::optional<Vec3> &out,
                         const std::string &label) {
        const bool any = a.comp_seen[0] || a.comp_seen[1] || a.comp_seen[2];
        if (!any)
            return;
        if (!(a.comp_seen[0] && a.comp_seen[1] && a.comp_seen[2]))
            throw std::invalid_argument(
                "set_tire_state: partial component columns for '" +
                std::string(label) + "'");
        // All-NaN -> unset (the artifact encodes None that way); a real
        // array can carry a stray NaN and still decodes as a vector.
        if (std::isnan(a.comp[0]) && std::isnan(a.comp[1]) &&
            std::isnan(a.comp[2])) {
            out = std::nullopt;
            return;
        }
        if (!std::isfinite(a.comp[0]) || !std::isfinite(a.comp[1]) ||
            !std::isfinite(a.comp[2]))
            throw std::invalid_argument(
                "set_tire_state: '" + std::string(label) +
                "' components must be finite or all-NaN");
        out = Vec3{a.comp[0], a.comp[1], a.comp[2]};
    };
    for (std::size_t s = 0; s < 2; ++s) {
        const std::string side = s == 0 ? "front" : "rear";
        fold(tangent[s], next[s].tangent, side + ".tangent");
        fold(point[s], next[s].point, side + ".point");
        fold(center[s], next[s].center, side + ".center");
        // Sentinel group: a stored tangent is only meaningful with the
        // contact point and profile segment it was produced from — the
        // Python oracle dereferences both unconditionally (TypeError).
        if (next[s].tangent && (!next[s].point || !next[s].segment)) {
            std::string message = "set_tire_state: '" + side;
            message += ".tangent' requires '" + side;
            message += ".point' and '" + side;
            message += ".segment'";
            throw std::invalid_argument(message);
        }
    }
    prepared_states_ = next;
}

void TireWriter::commit_prepared_state() noexcept {
    states_ = prepared_states_;
    // A restored state has not advanced under any interval yet — mirrors
    // `applier.last_time_s = None` in the Python restore path.
    last_time_s_ = std::nullopt;
}

std::vector<double> TireWriter::state() const {
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> out;
    out.reserve(22);
    const auto push_vec = [&out, nan_v](const std::optional<Vec3> &v) {
        for (std::size_t i = 0;
             i < static_cast<std::size_t>(model_access::kXYZ); ++i)
            out.push_back(v ? (*v)[i] : nan_v);
    };
    for (const BrushState &st: states_) {
        out.push_back(st.xi);
        push_vec(st.tangent);
        push_vec(st.point);
        out.push_back(st.segment
                          ? static_cast<double>(*st.segment)
                          : nan_v);
        push_vec(st.center);
    }
    return out;
}

// Caller precondition: `d` is the live mjData of the model this writer
// was built on, driven by the single owning Stepper — no model/data
// pairing check is derivable from widths here.
void TireWriter::compute_into(const mjData *d, double dt,
                              std::span<double> out) {
    if (d == nullptr)
        throw std::invalid_argument("tire compute needs a live mjData");
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()))
        throw std::invalid_argument("tire output width");
    accumulate_validated(d, dt, out);
}

// The warm-core twin of compute_into: the same precondition gates plus
// the interval/double-advance gates report CoreStatus::invalid_input
// instead of throwing, and residual kernel rejections are caught and
// mapped so no exception escapes — noexcept is proved by the catch-all.
// The kernel's own throw construction stays on the failure path only.
// biketyre::ProfileQuery::contact now refills its construction-sized
// member scratch, so a valid tick allocates nothing end to end and this
// entry carries BIKE_NONBLOCKING (declared in tire.hpp).
CoreStatus TireWriter::try_compute_into(const mjData *d, double dt,
                                        std::span<double> out) noexcept {
    if (d == nullptr || out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()) ||
        !std::isfinite(dt) || dt <= 0.0 ||
        (last_time_s_.has_value() &&
         static_cast<double>(d->time) <= *last_time_s_))
        return CoreStatus::invalid_input;
    try {
        accumulate_validated(d, dt, out);
    } catch (const std::bad_alloc &) {
        return CoreStatus::engine_failure;
    } catch (const engine::EngineFailure &) {
        // EngineFailure derives std::exception — it must be caught before
        // the generic mapping classifies an engine error as bad input.
        return CoreStatus::engine_failure;
    } catch (const std::exception &) {
        return CoreStatus::invalid_input;
    } catch (...) {
        return CoreStatus::engine_failure;
    }
    return CoreStatus::ok;
}

std::vector<double> TireWriter::qfrc(const mjData *d, double dt) {
    compute_into(d, dt, qfrc_out_);
    return qfrc_out_;
}

// Extracted unchanged from the original qfrc() body — including the
// interval/double-advance gates, the model-table require_id guards, and
// the derived() overflow gates, whose exception types are part of the
// observable contract. The staged snapshot/diagnostics locals became the
// persistent stage members; the commit at the end is unchanged.
void TireWriter::accumulate_validated(const mjData *d, double dt_arg,
                                      std::span<double> out) {
    // tire_forces.py:125-131 — scalar(dt,'tire interval',positive=True)
    // and the double-advance gate precede all force work.
    if (!std::isfinite(dt_arg) || dt_arg <= 0.0)
        throw std::invalid_argument("invalid tire interval");
    const double dt = dt_arg;
    const double time = static_cast<double>(d->time);
    if (last_time_s_.has_value() && time <= *last_time_s_)
        throw std::invalid_argument(
            "tire state can advance only once per increasing timestamp");
    // backend != 'compliant_2d' is rejected by the config reader, so the
    // zero-return path (line 129-131) is unreachable here.
    std::ranges::fill(out, 0.0);
    // The TireSpec materials engage iff the backend is compliant_2d — the
    // ctor's gate established it; restating once per call keeps the
    // per-contact optional accesses below checked without a per-side
    // branch. An invariant break is a logic error, not bad caller input.
    if (!cfg_.front.material || !cfg_.rear.material)
        throw std::logic_error(
            "compliant_2d tire params carry no material");
    const nativecfg::TireMaterial &front_mat = *cfg_.front.material;
    const nativecfg::TireMaterial &rear_mat = *cfg_.rear.material;
    double energy = 0.0, loss = 0.0, radial_loss_power = 0.0;
    std::array<BrushState, 2> new_states;
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
        d->geom_xpos, model_access::kXYZ * m_->ngeom);
    const std::span<const mjtNum> qvel =
            model_access::readonly_buffer(d->qvel, m_->nv);

    struct CompliantCpuIo {
        TireWriter *self = nullptr;
        double time_val = 0.0;
        double dt_val = 0.0;
        const nativecfg::TireMaterial *f_mat = nullptr;
        const nativecfg::TireMaterial *r_mat = nullptr;
        double *energy_acc = nullptr;
        double *loss_acc = nullptr;
        double *radial_power_acc = nullptr;
        std::string_view custom_material_name{};
        std::exception_ptr *caught = nullptr;

        bool eval_radial(
            bike::physics_core::i32 side,
            const bike::physics_core::RadialEvalInput<double> &in,
            bike::physics_core::RadialEvalOutput<double> &out_val,
            bike::physics_core::Status &out_status,
            bike::physics_core::ErrCode &err
        ) noexcept {
            const auto &radial_table =
                self->radial_tables_[static_cast<std::size_t>(side)];
            if (!radial_table) {
                return false;
            }
            try {
                const auto [norm_val, rad_e] = radial_table->normal_contact(
                    in.delta, in.delta_dot, in.eta, in.eps);
                out_val.normal = norm_val;
                out_val.energy = rad_e;
                out_val.elastic =
                    radial_table->elastic_response(in.delta).first;
                out_status = bike::physics_core::Status::ok;
                err = bike::physics_core::ErrCode::none;
            } catch (...) {
                *caught = std::current_exception();
                out_status = bike::physics_core::Status::invalid_argument;
                err = bike::physics_core::ErrCode::overflow_nonfinite;
            }
            return true;
        }

        bool custom_friction(
            const bike::physics_core::FrictionQuery<double> &query,
            bike::physics_core::FrictionResult<double> &out_val,
            bike::physics_core::Status &out_status,
            bike::physics_core::ErrCode &err
        ) noexcept {
            if (self->core_surface_map_complete_) {
                return false;
            }
            try {
                const auto [mu_val, mat_name] = tirecommon::effective_friction(
                    query.tire_mu,
                    self->surface_map_ ? &*self->surface_map_ : nullptr,
                    query.contact_x, query.slip);
                out_val.mu = mu_val;
                out_val.section_idx = -1;
                custom_material_name = mat_name;
                out_status = bike::physics_core::Status::ok;
                err = bike::physics_core::ErrCode::none;
            } catch (...) {
                *caught = std::current_exception();
                out_status = bike::physics_core::Status::invalid_argument;
                err = bike::physics_core::ErrCode::outside_domain;
            }
            return true;
        }

        bike::physics_core::Status record(
            bike::physics_core::i32 side,
            const bike::physics_core::CompliantRecord<double> &rec,
            bike::physics_core::ErrCode &err
        ) noexcept {
            try {
                const std::size_t s_idx = static_cast<std::size_t>(side);
                const nativecfg::TireParams &pcfg =
                    s_idx == 0 ? self->cfg_.front : self->cfg_.rear;
                const nativecfg::TireMaterial &mat =
                    s_idx == 0 ? *f_mat : *r_mat;
                const double tk = pcfg.tangent_k_n_m;
                const Vec3 center = {
                    rec.center.x, rec.center.y, rec.center.z};
                const Vec3 p = {rec.point.x, rec.point.y, rec.point.z};
                const Vec3 n = {rec.normal.x, rec.normal.y, rec.normal.z};

                std::string_view material_name = "configured";
                if (!self->core_surface_map_complete_) {
                    material_name = custom_material_name;
                } else if (self->surface_map_) {
                    material_name = rec.section_idx >= 0
                        ? std::string_view(
                              self->surface_map_
                                  ->sections[static_cast<std::size_t>(
                                      rec.section_idx)]
                                  .surface.name)
                        : std::string_view(self->surface_map_->surface.name);
                }

                TireSnapshot &snap = self->snapshot_stage_[s_idx];
                snap.patches.clear();
                snap.time_s = time_val;
                snap.backend = "compliant_2d";
                snap.geometric_contact = rec.contact.delta > 0.0;
                snap.wheel_axis_m = center;
                if (rec.contact.delta > 0.0) {
                    if (snap.patches.size() >= kMaxSnapshotPatches)
                        throw std::invalid_argument(
                            "tire snapshot patch capacity exceeded");
                    snap.patches.push_back(
                        make_patch(p, n, rec.normal_load, rec.tangent_force,
                                   rec.slip));
                }
                snap.interval_id =
                    interval_clock::interval_id(time_val, dt_val);
                if (!std::isfinite(time_val) || time_val < 0.0)
                    throw std::invalid_argument(
                        "contact snapshot needs finite nonnegative time");
                if (snap.interval_id < 0)
                    throw std::invalid_argument(
                        "contact snapshot needs a nonnegative interval ID");
                for (const double v: center)
                    if (!std::isfinite(v))
                        throw std::invalid_argument(
                            "wheel axis must be a finite 3D vector");
                snap.effective_radius_m = effective_radius(snap.patches, center);

                const double radial_loss =
                    (rec.normal_load - rec.elastic_force) * rec.delta_dot;
                *radial_power_acc +=
                    rec.contact.delta > 0.0 ? std::max(radial_loss, 0.0) : 0.0;
                *energy_acc +=
                    rec.radial_energy + 0.5 * tk * rec.xi_new * rec.xi_new;
                *loss_acc +=
                    std::max(rec.release_loss, 0.0) + rec.brush_loss;

                TireDiagnostics &diag = self->diagnostics_stage_[s_idx];
                diag.multi_support = rec.contact.multi_support != 0;
                diag.penetration_m = rec.contact.delta;
                diag.normal_speed_mps = -rec.delta_dot;
                diag.slip_mps = rec.slip;
                diag.normal_load_n = rec.normal_load;
                diag.tangent_force_n = rec.tangent_force;
                diag.friction_coefficient = rec.mu;
                diag.surface = material_name;
                diag.branch_release_loss_j = std::max(rec.release_loss, 0.0);
                diag.brush_loss_j = rec.brush_loss;
                diag.radial_energy_j = rec.radial_energy;
                diag.shear_energy_j = 0.5 * tk * rec.xi_new * rec.xi_new;
                diag.outside_material_load_range =
                    !nativecfg::is_load_in_valid_range(mat, rec.normal_load);
                err = bike::physics_core::ErrCode::none;
                return bike::physics_core::Status::ok;
            } catch (...) {
                *caught = std::current_exception();
                err = bike::physics_core::ErrCode::overflow_nonfinite;
                return bike::physics_core::Status::invalid_argument;
            }
        }
    };

    std::exception_ptr caught_err;
    auto terrain_fn = [&](double cx, double cz, double radius,
                          bool has_prev_seg, bike::physics_core::i32 prev_seg,
                          bike::physics_core::PolylineContactResult<double> &out_contact,
                          bike::physics_core::ErrCode &err) noexcept
        -> bike::physics_core::Status {
        try {
            const biketyre::ProfileContact contact = engaged(profile_).contact(
                {cx, cz}, radius,
                has_prev_seg ? std::optional<int>(prev_seg) : std::nullopt);
            out_contact.point[0] = contact.point[0];
            out_contact.point[1] = contact.point[1];
            out_contact.normal[0] = contact.normal[0];
            out_contact.normal[1] = contact.normal[1];
            out_contact.delta = contact.delta;
            out_contact.segment_id = contact.segment_id;
            out_contact.multi_support = contact.multi_support ? 1 : 0;
            err = bike::physics_core::ErrCode::none;
            return bike::physics_core::Status::ok;
        } catch (...) {
            caught_err = std::current_exception();
            err = bike::physics_core::ErrCode::outside_domain;
            return bike::physics_core::Status::invalid_argument;
        }
    };

    auto jac_fn = [&](bike::physics_core::i32 side,
                      const bike::physics_core::CompliantJacInput<double> &in_j,
                      const double * /*qvel_ptr*/,
                      bike::physics_core::CompliantJacOutput<double> &out_j,
                      bike::physics_core::ErrCode &err) noexcept
        -> bike::physics_core::Status {
        try {
            const int body = bodies_[static_cast<std::size_t>(side)];
            const Vec3 p_vec = {in_j.point.x, in_j.point.y, in_j.point.z};
            const Vec3 c_vec = {in_j.center.x, in_j.center.y, in_j.center.z};
            point_jacobian_into(m_, d, body, p_vec, jac_contact_);
            point_jacobian_into(m_, d, body, c_vec, jac_center_);
            std::array<double, 3> v_arr{};
            std::array<double, 3> cv_arr{};
            bike::physics_core::CpuExactToolkit::gemv_n3(
                nv_, jac_contact_.data(), qvel.data(), v_arr.data());
            bike::physics_core::CpuExactToolkit::gemv_n3(
                nv_, jac_center_.data(), qvel.data(), cv_arr.data());
            out_j.velocity = {.x = v_arr[0], .y = v_arr[1], .z = v_arr[2]};
            out_j.center_velocity = {.x = cv_arr[0], .y = cv_arr[1], .z = cv_arr[2]};
            err = bike::physics_core::ErrCode::none;
            return bike::physics_core::Status::ok;
        } catch (...) {
            caught_err = std::current_exception();
            err = bike::physics_core::ErrCode::out_of_bounds_chain;
            return bike::physics_core::Status::invalid_argument;
        }
    };

    auto qfrc_accum_fn = [&](bike::physics_core::i32 /*side*/,
                             const bike::physics_core::TyreVec3<double> & /*p_v*/,
                             const bike::physics_core::TyreVec3<double> &fw_v,
                             double * /*out_ptr*/,
                             bike::physics_core::ErrCode &err) noexcept
        -> bike::physics_core::Status {
        const std::array<double, 3> fw_arr = {fw_v.x, fw_v.y, fw_v.z};
        bike::physics_core::CpuExactToolkit::gemv_t3(
            nv_, jac_contact_.data(), fw_arr.data(), tmp_.data());
        for (int i = 0; i < nv_; ++i) {
            out[static_cast<std::size_t>(i)] += tmp_[static_cast<std::size_t>(i)];
        }
        err = bike::physics_core::ErrCode::none;
        return bike::physics_core::Status::ok;
    };

    CompliantCpuIo cpu_io{
        .self = this,
        .time_val = time,
        .dt_val = dt,
        .f_mat = &front_mat,
        .r_mat = &rear_mat,
        .energy_acc = &energy,
        .loss_acc = &loss,
        .radial_power_acc = &radial_loss_power,
        .custom_material_name = {},
        .caught = &caught_err
    };

    for (std::size_t s = 0; s < 2; ++s) {
        const BrushState &state = states_[s];
        const std::size_t g3 =
                static_cast<std::size_t>(model_access::kXYZ) *
                static_cast<std::size_t>(geoms_[s]);
        const Vec3 center = {xpos[g3], xpos[g3 + 1], xpos[g3 + 2]};
        bike::physics_core::CompliantWheelStatePod<double> prev_pod{};
        prev_pod.xi = state.xi;
        if (state.tangent) {
            std::ranges::copy(*state.tangent, std::span{prev_pod.tangent}.begin());
            prev_pod.has_tangent = 1;
        }
        if (state.point) {
            std::ranges::copy(*state.point, std::span{prev_pod.point}.begin());
            prev_pod.has_point = 1;
        }
        if (state.center) {
            std::ranges::copy(*state.center, std::span{prev_pod.center}.begin());
            prev_pod.has_center = 1;
        }
        if (state.segment) {
            prev_pod.segment = *state.segment;
            prev_pod.has_segment = 1;
        }

        bike::physics_core::CompliantWheelStatePod<double> next_pod{};
        auto err = bike::physics_core::ErrCode::none;
        const auto st = bike::physics_core::step_compliant_wheel<
            double, bike::physics_core::CpuExactToolkit>(
                core_model_.get(),
                static_cast<bike::physics_core::i32>(s),
                dt,
                center.data(),
                qvel.data(),
                &prev_pod,
                &next_pod,
                out.data(),
                terrain_fn,
                jac_fn,
                qfrc_accum_fn,
                cpu_io,
                err);
        if (st != bike::physics_core::Status::ok) {
            if (caught_err) {
                std::rethrow_exception(caught_err);
            }
            if (err == bike::physics_core::ErrCode::invalid_prev_seg) {
                throw std::invalid_argument(
                    "restored tire state has tangent without "
                    "segment/point");
            }
            if (err == bike::physics_core::ErrCode::overflow_passivity) {
                throw std::overflow_error(
                    "brush update violates discrete passivity");
            }
            if (st == bike::physics_core::Status::overflow) {
                contactlaw::finite_fail();
            }
            if (st == bike::physics_core::Status::logic_error) {
                throw std::logic_error("compliant wheel logic error");
            }
            throw std::invalid_argument("compliant wheel invalid argument");
        }
        new_states[s] = {
            .xi = next_pod.xi,
            .tangent = Vec3{next_pod.tangent[0], next_pod.tangent[1], next_pod.tangent[2]},
            .point = Vec3{next_pod.point[0], next_pod.point[1], next_pod.point[2]},
            .segment = next_pod.segment,
            .center = Vec3{next_pod.center[0], next_pod.center[1], next_pod.center[2]}
        };
    }
    // Commit persistent state only after both wheels evaluate successfully
    // (tire_forces.py:201-205). The stage members copy-assign into the
    // committed slots — capacity-reserved at construction, so the marked
    // path never allocates — and `committed_` flips the observable
    // published state. A throwing evaluation still never mutates the
    // published state.
    validation::derived_array(out, "TireWriter.qfrc");
    for (const double value: {energy, loss, radial_loss_power})
        validation::derived(value, "TireWriter.energy_or_power");
    states_ = new_states;
    for (std::size_t s = 0; s < 2; ++s) {
        snapshots_[s] = snapshot_stage_[s];
        diagnostics_[s] = diagnostics_stage_[s];
    }
    committed_ = true;
    elastic_energy_j_ = energy;
    brush_loss_step_j_ = loss;
    radial_dissipation_power_w_ = radial_loss_power;
    last_time_s_ = time;
}

// stored_energy (tire_forces.py:173-184): the per-side material energy
// under the CURRENT geometry plus the stored brush shear. The config
// re-validation (`self.config.__post_init__()`) is a no-op here — the
// writer's config is const and was validated at construction; nothing in
// this TU can mutate it.
double TireWriter::stored_energy(const mjData *d) const {
    if (d == nullptr)
        throw std::invalid_argument(
            "tire stored_energy needs a live mjData");
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
        d->geom_xpos, model_access::kXYZ * m_->ngeom);
    // Same engaged-material invariant as the apply path — the ctor's
    // compliant_2d gate plus validate() established it.
    if (!cfg_.front.material || !cfg_.rear.material)
        throw std::logic_error(
            "compliant_2d tire params carry no material");
    const nativecfg::TireMaterial &front_mat = *cfg_.front.material;
    const nativecfg::TireMaterial &rear_mat = *cfg_.rear.material;
    double energy = 0.0;
    for (std::size_t s = 0; s < 2; ++s) {
        const nativecfg::TireParams &cfg = s == 0 ? cfg_.front : cfg_.rear;
        const nativecfg::TireMaterial &mat = s == 0 ? front_mat : rear_mat;
        const BrushState &state = states_[s];
        const std::size_t g3 =
                static_cast<std::size_t>(model_access::kXYZ) *
                static_cast<std::size_t>(geoms_[s]);
        const biketyre::ProfileContact contact = engaged(profile_).contact(
            {xpos[g3], xpos[g3 + 2]}, radii_[s], state.segment);
        // TireSpec.elastic_response (tire.py:73-79): _scalar(delta) then
        // max(0,·), force and energy both validated by _finite_result.
        if (!std::isfinite(contact.delta))
            throw std::invalid_argument("tire deflection must be finite");
        const double penetration = std::max(0.0, contact.delta);
        const auto& radial_table = radial_tables_[s];
        const auto table_response = radial_table
            ? radial_table->elastic_response(contact.delta) : std::pair<double, double>{};
        const double force = radial_table ? table_response.first : mat.radial_k_n_m * penetration;
        const double elastic = radial_table ? table_response.second :
            0.5 * mat.radial_k_n_m * pyfloat::pow(penetration, 2.0);
        validation::derived(force, "TireSpec.elastic_response");
        validation::derived(elastic, "TireSpec.elastic_response");
        energy += elastic;
        energy += 0.5 * cfg.tangent_k_n_m *
                  pyfloat::pow(state.xi, 2.0);
    }
    return validation::derived(energy, "TireForceApplier.stored_energy");
}

// compute_qfrc(advance=False) (tire_forces.py:108-113): a detached copy
// runs the full advancing kernel on a cleared clock; only its committed
// snapshots cross back. The copy duplicates scratch the oracle shares —
// observably identical, and the probe never mutates the caller.
void TireWriter::probe_compute_into(const mjData *d, double dt,
                                    std::span<double> out) {
    if (d == nullptr)
        throw std::invalid_argument("tire probe needs a live mjData");
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()))
        throw std::invalid_argument("tire output width");
    TireWriter probe(*this);
    probe.last_time_s_.reset();
    probe.accumulate_validated(d, dt, out);
    probe_snapshots_ = probe.snapshots_;
    probe_committed_ = probe.committed_;
}

std::vector<double> TireWriter::probe_qfrc(const mjData *d, double dt) {
    std::vector<double> out(static_cast<std::size_t>(nv_));
    probe_compute_into(d, dt, out);
    return out;
}
