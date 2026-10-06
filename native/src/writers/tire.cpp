// writers/tire.cpp — expression-for-expression port of tire_forces.py's
// compute path plus physics/tire.py's material laws and the contact_state.py
// snapshot/patch validation. Comments cite file:line at porting time.
//
// The same-libcall argument from writers/resistance.cpp applies everywhere
// a `@`/`np.dot`/`np.linalg.norm` appears: numpy resolves those to Apple
// Accelerate's ddot/dgemv, and this TU invokes the same CBLAS symbols.
// `np.einsum('ij,ij->i')` is NOT a BLAS call — it is numpy's seeded
// 0.0+p0+p1 accumulation — replicated literally. Python `sum()` gets the
// Neumaier compensation; `round()` is std::nearbyint under the default
// to-nearest-even mode; `min`/`max` are std::min/std::max (identical
// `b<a?b:a` / `a<b?b:a` semantics, NaN included).
#include "tire.hpp"
#include "../config_validation.hpp"

#include "../contact/laws.hpp"
#include "../diag.hpp"
#include "../engaged.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

extern "C" {
// Legacy CBLAS ABI — see writers/resistance.cpp / tyre/profile.cpp.
double cblas_ddot(int N, const double *X, int incX, const double *Y,
                  int incY);

void cblas_dgemv(int Order, int TransA, int M, int N, double alpha,
                 const double *A, int lda, const double *X, int incX,
                 double beta, double *Y, int incY);
}

namespace {
    constexpr int kCblasRowMajor = 101;
    constexpr int kCblasNoTrans = 111;
    constexpr int kCblasTrans = 112;

    using Vec3 = std::array<double, 3>;

    using contactlaw::normal_contact;
    using contactlaw::brush_step;

    // physical_mapping.py:6-11 — resolve_id.
    int resolve_id(const mjModel *m, mjtObj kind, const char *name) {
        const int result = mj_name2id(m, kind, name);
        if (result < 0)
            throw std::invalid_argument("model has no '" + std::string(name) +
                                        "'");
        return result;
    }

    // physical_mapping.py:24-37 — point_jacobian_into (jacp only).
    void point_jacobian_into(const mjModel *m, const mjData *d, int body_id,
                             const Vec3 &point, mjtNum *jp) {
        if (!std::ranges::all_of(point,
                                 [](double v) { return std::isfinite(v); }))
            throw std::invalid_argument("invalid world point");
        if (!(0 < body_id && body_id < m->nbody))
            throw std::invalid_argument(
                "point Jacobian requires a physical body");
        mj_jac(m, d, jp, nullptr, point.data(), body_id);
    }

    // np.linalg.norm(v) on a (3,) — sqrt(ddot(v,v)), the same-libcall form.
    double norm3(const Vec3 &v) {
        return std::sqrt(cblas_ddot(3, v.data(), 1, v.data(), 1));
    }

    Vec3 sub(const Vec3 &a, const Vec3 &b) {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    double dot3(const Vec3 &a, const Vec3 &b) {
        return cblas_ddot(3, a.data(), 1, b.data(), 1);
    }

    // CPython builtin sum() on floats: Neumaier compensated summation — see
    // writers/resistance.cpp for the derivation. `sum(iterable)` with no
    // explicit start seeds int 0 (compares equal to 0.0 downstream).
    double py_sum(std::span<const double> values) {
        double f_result = 0.0;
        double c_sum = 0.0;
        for (const double x: values) {
            const double t = f_result + x;
            if (std::isfinite(t)) {
                if (std::abs(f_result) >= std::abs(x))
                    c_sum += (f_result - t) + x;
                else
                    c_sum += (x - t) + f_result;
            }
            f_result = t;
        }
        return f_result + c_sum;
    }

    // surface.py:72-88 — SurfaceSpec.mu: mu_slide + (peak-slide)*exp(-|v|/str).
    double surface_mu(const nativecfg::SurfaceSpec &s, double slip) {
        const double decay =
                std::exp(-std::abs(slip) / s.stribeck_speed_mps);
        return s.mu_slide + (s.mu_peak - s.mu_slide) * decay;
    }

    // surface.py:194-206 — SurfaceMap.at: finite query, first matching
    // half-open section wins, else the default material.
    const nativecfg::SurfaceSpec &
    surface_at(const nativecfg::SurfaceMap &map, double x_m) {
        if (!std::isfinite(x_m))
            throw std::invalid_argument("material query must be finite");
        for (const auto &sec: map.sections)
            if (sec.start_m <= x_m && x_m < sec.end_m)
                return sec.surface;
        return map.surface;
    }

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
            .working_surface = true
        };
    }

    NATIVE_DIAG_POP

    // WheelContactSnapshot.effective_radius_m (contact_state.py:172-181).
    double effective_radius(const std::vector<TirePatch> &patches,
                            const Vec3 &axis) {
        if (patches.empty())
            return 0.0;
        const bool any_loaded = std::ranges::any_of(
            patches, [](const TirePatch &p) { return p.normal_load_n > 0.0; });
        std::vector<double> terms, weights;
        terms.reserve(patches.size());
        weights.reserve(patches.size());
        for (const TirePatch &p: patches) {
            const double w = any_loaded ? p.normal_load_n : 1.0;
            // float(np.dot(p.point_m - axis, p.normal)) → ddot
            const double drop = -dot3(sub(p.point_m, axis), p.normal);
            terms.push_back(w * drop);
            weights.push_back(w);
        }
        return std::max(0.0, py_sum(terms) / py_sum(weights));
    }
} // namespace

TireWriter::TireWriter(const mjModel *m, const mjData *d,
                       nativecfg::TireConfig config)
    : m_(m), cfg_(std::move(config)), nv_(static_cast<int>(m->nv)) {
    nativecfg::validate(cfg_);
    // Dataclass __post_init__ validation happened when Python built the
    // config — before TireForceApplier.__init__ — so these checks precede
    // the ctor body like they do in the Python flow.
    // TireSpec.__post_init__ (tire.py:52-69) on both sides.
    for (const nativecfg::TireParams *p: {&cfg_.front, &cfg_.rear}) {
        const nativecfg::TireMaterial &mat = p->material;
        if (!std::isfinite(mat.radial_k_n_m) ||
            !std::isfinite(mat.radial_c_ns_m) ||
            !std::isfinite(mat.pressure_pa_gauge))
            throw std::invalid_argument(
                "tire material scalars must be finite");
        if (mat.radial_k_n_m <= 0.0)
            throw std::invalid_argument("radial_k_n_m must be positive");
        if (mat.radial_c_ns_m < 0.0 || mat.pressure_pa_gauge < 0.0)
            throw std::invalid_argument(
                "radial damping and gauge pressure must be nonnegative");
        if (mat.provenance.find_first_not_of(" \t\n\v\f\r") ==
            std::string::npos)
            throw std::invalid_argument(
                "provenance must be a nonempty description, e.g. synthetic");
        const auto [lo, hi] = mat.valid_load_range_n;
        if (!std::isfinite(lo) || !std::isfinite(hi) ||
            !(0.0 <= lo && lo < hi))
            throw std::invalid_argument(
                "valid load bounds must satisfy 0 <= minimum < maximum");
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
            std::views::counted(m->geom_bodyid, m->ngeom);
    const std::span<const mjtNum> gsize =
            std::views::counted(m->geom_size, 3 * m->ngeom);
    for (std::size_t i = 0; i < 2; ++i) {
        bodies_[i] = body_of[static_cast<std::size_t>(geoms_[i])];
        radii_[i] = gsize[3 * static_cast<std::size_t>(geoms_[i])];
    }
    jac_contact_.resize(static_cast<std::size_t>(3) *
                        static_cast<std::size_t>(nv_));
    jac_center_.resize(static_cast<std::size_t>(3) *
                       static_cast<std::size_t>(nv_));
    // tire_forces.py:84-88 — every geom on each wheel body must be
    // collision-disabled for the compliant_2d backend.
    const std::span<const int> contype =
            std::views::counted(m->geom_contype, m->ngeom);
    const std::span<const int> conaffinity =
            std::views::counted(m->geom_conaffinity, m->ngeom);
    for (const int body: bodies_)
        for (int g = 0; g < m->ngeom; ++g)
            if (body_of[static_cast<std::size_t>(g)] == body &&
                (contype[static_cast<std::size_t>(g)] ||
                 conaffinity[static_cast<std::size_t>(g)]))
                throw std::invalid_argument(
                    "native wheel collisions must be disabled for "
                    "compliant_2d");
    // reset() (tire_forces.py:91-98) — the members' initializers already
    // hold these values; no snapshots/diagnostics until the first compute.
}

void TireWriter::reset() {
    states_ = {};
    snapshots_.fill(std::nullopt);
    diagnostics_.fill(std::nullopt);
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
        std::array<bool, 3> comp_seen = {false, false, false};
        std::array<double, 3> comp = {0.0, 0.0, 0.0};
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
        if (next[s].tangent && (!next[s].point || !next[s].segment))
            throw std::invalid_argument(
                "set_tire_state: '" + side +
                ".tangent' requires '" + side + ".point' and '" + side +
                ".segment'");
    }
    states_ = next;
    // A restored state has not advanced under any interval yet — mirrors
    // `applier.last_time_s = None` in the Python restore path.
    last_time_s_ = std::nullopt;
}

std::vector<double> TireWriter::state() const {
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> out;
    out.reserve(22);
    const auto push_vec = [&out, nan_v](const std::optional<Vec3> &v) {
        for (std::size_t i = 0; i < 3; ++i)
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

std::vector<double> TireWriter::qfrc(const mjData *d, double dt_arg) {
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
    std::vector<double> qfrc(static_cast<std::size_t>(nv_), 0.0);
    double energy = 0.0, loss = 0.0, radial_loss_power = 0.0;
    std::array<BrushState, 2> new_states;
    std::array<std::optional<TireSnapshot>, 2> snapshots;
    std::array<std::optional<TireDiagnostics>, 2> diagnostics;
    const std::span<const mjtNum> xpos =
            std::views::counted(d->geom_xpos, 3 * m_->ngeom);
    const std::span<const mjtNum> qvel =
            std::views::counted(d->qvel, m_->nv);
    const double normal_cosine = std::cos(
        cfg_.distinct_normal_deg * (std::numbers::pi / 180.0));
    std::vector<double> tmp(static_cast<std::size_t>(nv_));
    for (std::size_t s = 0; s < 2; ++s) {
        const nativecfg::TireParams &pcfg =
                s == 0 ? cfg_.front : cfg_.rear;
        const BrushState &state = states_[s];
        const int geom = geoms_[s], body = bodies_[s];
        const std::size_t g3 = 3 * static_cast<std::size_t>(geom);
        const double radius = radii_[s];
        const double tk = pcfg.tangent_k_n_m;
        // center = np.array(data.geom_xpos[geom], copy=True)
        const Vec3 center = {xpos[g3], xpos[g3 + 1], xpos[g3 + 2]};
        const biketyre::ProfileContact contact = engaged(profile_).contact(
            {center[0], center[2]}, radius, state.segment);
        const Vec3 p = {contact.point[0], 0.0, contact.point[1]};
        const Vec3 n = {contact.normal[0], 0.0, contact.normal[1]};
        const Vec3 tangent = {n[2], 0.0, -n[0]};
        point_jacobian_into(m_, d, body, p, jac_contact_.data());
        point_jacobian_into(m_, d, body, center, jac_center_.data());
        // velocity = jac @ qvel / center_velocity = jac_center @ qvel
        Vec3 velocity, center_velocity;
        cblas_dgemv(kCblasRowMajor, kCblasNoTrans, 3, nv_, 1.0,
                    jac_contact_.data(), nv_, qvel.data(), 1, 0.0,
                    velocity.data(), 1);
        cblas_dgemv(kCblasRowMajor, kCblasNoTrans, 3, nv_, 1.0,
                    jac_center_.data(), nv_, qvel.data(), 1, 0.0,
                    center_velocity.data(), 1);
        const double delta_dot = -dot3(velocity, n);
        const double slip = dot3(velocity, tangent);
        // TireSpec material branch (tire_forces.py:147-150).
        const double rk = pcfg.material.radial_k_n_m;
        const double rc = pcfg.material.radial_c_ns_m;
        const auto [normal, radial_energy] =
                normal_contact(contact.delta, delta_dot, rk, rc);
        const double elastic_force = rk * std::max(contact.delta, 0.0);
        double xi = state.xi;
        double release_loss = 0.0;
        if (state.tangent.has_value()) {
            // Python reads state.segment/state.point unconditionally in
            // this branch — a tangent without them is a malformed restore
            // (numpy would raise TypeError on int-None).
            if (!state.segment.has_value() || !state.point.has_value())
                throw std::invalid_argument(
                    "restored tire state has tangent without "
                    "segment/point");
            const bool adjacent =
                    std::abs(contact.segment_id - *state.segment) <= 1;
            const bool coincident = norm3(sub(p, *state.point)) < 1e-8;
            const double moved =
                    state.center ? norm3(sub(center, *state.center)) : 0.0;
            const bool similar =
                    dot3(*state.tangent, tangent) >= normal_cosine;
            const bool continuous =
                    similar &&
                    norm3(sub(p, *state.point)) <=
                    2.0 * moved +
                    radius * norm3(sub(tangent, *state.tangent)) +
                    1e-8;
            if (!adjacent && !coincident && !continuous) {
                release_loss = 0.5 * tk * xi * xi;
                xi = 0.0;
            } else {
                const double transported =
                        xi * dot3(*state.tangent, tangent);
                release_loss =
                        0.5 * tk * (xi * xi - transported * transported);
                xi = transported;
            }
        }
        // effective_friction (tire_forces.py:56-61).
        double mu = std::numeric_limits<double>::quiet_NaN();
        std::string material_name;
        if (!surface_map_) {
            mu = pcfg.mu;
            material_name = "configured";
        } else {
            const nativecfg::SurfaceSpec &surf =
                    surface_at(*surface_map_, p[0]);
            mu = std::min(pcfg.mu, surface_mu(surf, slip));
            material_name = surf.name;
        }
        const auto [xi_new, force, brush_loss] = brush_step(
            xi, slip, dot3(center_velocity, tangent), normal, tk, mu,
            pcfg.relaxation_length_m, dt);
        // force_world = normal*n + force*tangent (two elementwise roundings)
        const Vec3 force_world = {
            normal * n[0] + force * tangent[0],
            normal * n[1] + force * tangent[1],
            normal * n[2] + force * tangent[2]
        };
        // qfrc += jac_contact.T @ force_world — the same gemv-trans path
        // the resistance writer uses, then elementwise +=.
        cblas_dgemv(kCblasRowMajor, kCblasTrans, 3, nv_, 1.0,
                    jac_contact_.data(), nv_, force_world.data(), 1, 0.0,
                    tmp.data(), 1);
        for (int i = 0; i < nv_; ++i)
            qfrc[static_cast<std::size_t>(i)] +=
                    tmp[static_cast<std::size_t>(i)];
        // WheelContactSnapshot args evaluate before __post_init__ checks:
        // the patch tuple (line 180) and round(time/dt) (line 183) run
        // first, then the time/interval validation.
        snapshots[s].emplace();
        TireSnapshot &snap = engaged(snapshots[s]);
        snap.time_s = time;
        snap.backend = "compliant_2d";
        snap.geometric_contact = contact.delta > 0.0;
        snap.wheel_axis_m = center;
        if (contact.delta > 0.0)
            snap.patches.push_back(
                make_patch(p, n, normal, force, slip));
        const double quotient = time / dt;
        if (!std::isfinite(quotient))
            throw std::overflow_error(
                "cannot convert float infinity to integer");
        const double rounded = std::nearbyint(quotient);
        if (std::abs(rounded) > 9.2e18)
            throw std::overflow_error(
                "cannot convert float infinity to integer");
        snap.interval_id = static_cast<std::int64_t>(rounded);
        // WheelContactSnapshot.__post_init__ (contact_state.py:90-113).
        if (!std::isfinite(time) || time < 0.0)
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
        // Continuous radial passivity (tire_forces.py:186-189).
        const double radial_loss = (normal - elastic_force) * delta_dot;
        radial_loss_power +=
                contact.delta > 0.0 ? std::max(radial_loss, 0.0) : 0.0;
        energy += radial_energy + 0.5 * tk * xi_new * xi_new;
        loss += std::max(release_loss, 0.0) + brush_loss;
        // diagnostics[side] (tire_forces.py:190-199).
        const TireDiagnostics diag = {
            .multi_support = contact.multi_support,
            .penetration_m = contact.delta,
            .normal_speed_mps = -delta_dot,
            .slip_mps = slip,
            .normal_load_n = normal,
            .tangent_force_n = force,
            .friction_coefficient = mu,
            .surface = material_name,
            .branch_release_loss_j = std::max(release_loss, 0.0),
            .brush_loss_j = brush_loss,
            .radial_energy_j = radial_energy,
            .shear_energy_j = 0.5 * tk * xi_new * xi_new,
            .outside_material_load_range = !(pcfg.material.valid_load_range_n[0] <= normal &&
                                             normal <= pcfg.material.valid_load_range_n[1]),
        };
        diagnostics[s] = diag;
        new_states[s] = {.xi = xi_new, .tangent = tangent, .point = p, .segment = contact.segment_id, .center = center};
    }
    // Commit persistent state only after both wheels evaluate successfully
    // (tire_forces.py:201-205).
    validation::derived_array(qfrc, "TireWriter.qfrc");
    for (const double value: {energy, loss, radial_loss_power})
        validation::derived(value, "TireWriter.energy_or_power");
    states_ = new_states;
    snapshots_ = std::move(snapshots);
    diagnostics_ = std::move(diagnostics);
    elastic_energy_j_ = energy;
    brush_loss_step_j_ = loss;
    radial_dissipation_power_w_ = radial_loss_power;
    last_time_s_ = time;
    return qfrc;
}
