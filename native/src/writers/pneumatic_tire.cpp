// writers/pneumatic_tire.cpp — port of pneumatic_tire_forces.py:
// construction gates, the canonical state wire, stored energy, the
// advancing two-wheel tick with model-state rollback, probe isolation,
// and the snapshot/diagnostics/energy commit. Comments cite
// file:line at porting time.
//
// NumPy/CPython lowering decisions (same-libcall contract, verified
// bitwise by tests/reference/test_native_pneumatic_tire.py):
//   jacp.T @ force                → cblas_dgemv(RowMajor, Trans) into
//                                   tmp_, then elementwise += — the
//                                   oracle materializes the product
//                                   before the +=; never fused
//   np.linalg.norm / np.dot on (3,) → blas::ddot (+ sqrt)
//   np.sum(row)                   → numeric::numpy_pairwise_sum
//   builtin sum(generator)        → numeric::PythonSum (Neumaier)
//   np.max(delta)                 → sequential np_maximum fold (the
//                                   maximum.reduce path; NaN propagates)
//   np.maximum(x, 0.0)            → numeric::np_maximum
//   x ** 2                        → pyfloat::float_pow — the libm call,
//                                   never the x*x fold
//   np.isclose(a, b, 0, atol)     → |a-b| <= atol (rtol term is 0)
//   float(data.time) etc          → static_cast<double>
//
// Rollback: the oracle deepcopies both PneumaticTyre objects before the
// wheel loop and rebinds them on any exception; the port keeps two
// construction-sized clone banks — backup_ is assign()ed before the
// loop, probe_ is the detached copy advance=False evaluates on — and
// restores by assign()ing back, which reproduces deepcopy+rebind
// bitwise without a per-tick allocation.
#include "pneumatic_tire.hpp"

#include "../cblas_abi.hpp"
#include "../config_validation.hpp"
#include "../diag.hpp"
#include "../engaged.hpp"
#include "../engine_call.hpp"
#include "../interval_clock.hpp"
#include "../model_access.hpp"
#include "../numeric_numpy.hpp"
#include "../numeric_sum.hpp"
#include "../pneumatic/geometry.hpp"
#include "pyfloat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <format>
#include <limits>
#include <new>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
    using Vec3 = std::array<double, model_access::kXYZ>;

    // _resolve_wheel (tyre/applier.py:161-192) — the body/joint/geom
    // name resolution in that order, the sphere gate, and the
    // isclose(rtol=0, atol=1e-6) radius cross-check. Messages keep the
    // oracle's repr() quoting verbatim.
    struct WheelBinding {
        int body_id = -1, spin_dofadr = -1, contact_geom_id = -1;
    };

    WheelBinding resolve_wheel(const mjModel *m, const char *body_name,
                               const char *joint_name,
                               const char *geom_name,
                               double expected_radius_m) {
        const int body_id = mj_name2id(m, mjOBJ_BODY, body_name);
        const int joint_id = mj_name2id(m, mjOBJ_JOINT, joint_name);
        const int geom_id = mj_name2id(m, mjOBJ_GEOM, geom_name);
        for (const auto &[id, name]:
             std::array<std::pair<int, const char *>, 3>{{
                 {body_id, body_name},
                 {joint_id, joint_name},
                 {geom_id, geom_name}}})
            if (id < 0)
                throw std::invalid_argument(
                        "model has no '" + std::string(name) +
                        "'; pneumatic tyre force path cannot run");
        if (model_access::readonly_buffer(m->geom_type, m->ngeom)
                    [static_cast<std::size_t>(geom_id)] != mjGEOM_SPHERE)
            throw std::invalid_argument(
                    "contact geom '" + std::string(geom_name) +
                    "' must remain a sphere");
        const double actual = static_cast<double>(
                model_access::readonly_buffer(
                        m->geom_size, model_access::kXYZ * m->ngeom)
                        [model_access::kXYZ *
                         static_cast<std::size_t>(geom_id)]);
        // np.isclose(actual, expected, rtol=0.0, atol=1e-6).
        if (!(std::abs(actual - expected_radius_m) <= 1e-6))
            throw std::invalid_argument(std::format(
                    "contact geom '{}' radius {:.6f} m does not match "
                    "tyre radius {:.6f} m",
                    geom_name, actual, expected_radius_m));
        return {.body_id = body_id,
                .spin_dofadr = static_cast<int>(
                        model_access::readonly_buffer(m->jnt_dofadr,
                                                      m->njnt)
                                [static_cast<std::size_t>(joint_id)]),
                .contact_geom_id = geom_id};
    }

    // np.linalg.norm(v) on a (3,) — sqrt(ddot(v,v)), the same-libcall
    // form the ContactPatch unit-normal gate reduces through.
    double norm3(const Vec3 &v) {
        return std::sqrt(blas::ddot(model_access::kXYZ, v.data(), 1,
                                    v.data(), 1));
    }

    double dot3(const Vec3 &a, const Vec3 &b) {
        return blas::ddot(model_access::kXYZ, a.data(), 1, b.data(), 1);
    }

    // ContactPatch.__post_init__ (contact_state.py:32-59) — source
    // 'terrain', no couple/native force. Verbatim copy of the tire.cpp /
    // distributed_tire.cpp helper — identical expressions, identical
    // accept/reject behaviour.
    NATIVE_DIAG_PUSH
    NATIVE_DIAG_IGNORE("-Wlarge-by-value-copy")
    TirePatch make_patch(Vec3 point, Vec3 normal, double load,
                         double force, double slip) {
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
            .point_m = point, .normal = normal, .normal_load_n = load,
            .tangent_force_n = force, .slip_mps = slip,
            .working_surface = true
        };
    }
    NATIVE_DIAG_POP

    // WheelContactSnapshot.effective_radius_m (contact_state.py:172-181):
    // the load-weighted mean contact drop; unit weights when every patch
    // is unloaded; 0 when there is no patch. numeric::PythonSum is the
    // builtin sum() Neumaier loop.
    double effective_radius(const std::vector<TirePatch> &patches,
                            const Vec3 &axis) {
        if (patches.empty())
            return 0.0;
        const bool any_loaded = std::ranges::any_of(
            patches, [](const TirePatch &p) {
                return p.normal_load_n > 0.0;
            });
        numeric::PythonSum terms, weights;
        for (const TirePatch &p: patches) {
            const double w = any_loaded ? p.normal_load_n : 1.0;
            // float(np.dot(p.point_m - axis, p.normal)) → ddot
            const Vec3 offset = {p.point_m[0] - axis[0],
                                 p.point_m[1] - axis[1],
                                 p.point_m[2] - axis[2]};
            const double drop = -dot3(offset, p.normal);
            terms.add(w * drop);
            weights.add(w);
        }
        return std::max(0.0, terms.total() / weights.total());
    }

    // _RAY_LEAVES / _RIM_LEAVES / _PATCH_LEAVES / _SIDES
    // (pneumatic_tire_forces.py:48-51), in wire order.
    constexpr std::array<const char *, 4> kRayLeaves = {
        "delta_prev_m", "elastic_prev_n", "maxwell_n", "bristle_m"};
    constexpr std::array<const char *, 6> kRimLeaves = {
        "active", "x_m", "speed_mps", "peak_load_n", "peak_force_n",
        "energy_j"};
    constexpr std::array<const char *, 3> kPatchLeaves = {
        "start", "stop", "kappa_prime"};
    constexpr std::array<const char *, 2> kSides = {"front", "rear"};
} // namespace

// Caller preconditions (documented at the raw boundary): `d` must be an
// mjData created from `m` — dimension equality is a width check, never a
// pairing proof — and a single owning context owns the writer for the
// model's lifetime so `m_`/`d` outlive every call.
PneumaticTireWriter::PneumaticTireWriter(const mjModel *m,
                                         const mjData *d,
                                         nativecfg::TireConfig config)
    : m_(m), cfg_(std::move(config)), nv_(static_cast<int>(m->nv)) {
    static_cast<void>(d); // __init__ reads no mjData fields either.
    if (m_->nv < 0)
        throw std::invalid_argument(
            "pneumatic tire writer needs a nonnegative model width");
    // Dataclass __post_init__ validation happened when Python built the
    // config — before __init__ — so validate() precedes the ctor body
    // (same convention as the distributed writer).
    nativecfg::validate(cfg_);
    // :78-82 — the config gates in the oracle's order.
    if (!cfg_.pneumatic)
        throw std::invalid_argument(
            "pneumatic tires need an explicit pneumatic config");
    if (cfg_.backend != "pneumatic")
        throw std::invalid_argument(
            "pneumatic force applier requires the pneumatic backend");
    const nativecfg::PneumaticConfig &pneu = engaged(cfg_.pneumatic);
    // :83-86 — the RoadProfile gate; the wire always carries the profile,
    // so the isinstance check lands on RoadProfile's own domain checks.
    profile_.emplace(pneu.profile);
    // :87-90 — SurfaceMap.uniform(pneumatic.surface) when a configured
    // surface shipped, else the track map; both must produce a map.
    if (pneu.surface.has_value()) {
        surface_map_.surface = *pneu.surface;
        surface_map_.sections.clear();
    } else if (cfg_.surface_map.has_value()) {
        // SurfaceMap.__init__ (surface.py:160-206) sorts sections by
        // start_m; spec/overlap checks are nativecfg::validate()'s.
        surface_map_ = *cfg_.surface_map;
        std::ranges::sort(surface_map_.sections, {},
                          [](const auto &sec) { return sec.start_m; });
    } else {
        throw std::invalid_argument(
            "pneumatic tires need a track surface map or a configured "
            "surface");
    }
    // :92-99 — self.wheels, resolved in dict-literal order.
    const WheelBinding front =
            resolve_wheel(m_, "front_wheel", "front_wheel_spin",
                          "geom_front_contact",
                          pneu.front.outer_radius_mm / 1000.0);
    const WheelBinding rear =
            resolve_wheel(m_, "rear_wheel", "rear_wheel_spin",
                          "geom_rear_contact",
                          pneu.rear.outer_radius_mm / 1000.0);
    bodies_ = {front.body_id, rear.body_id};
    spin_dofs_ = {front.spin_dofadr, rear.spin_dofadr};
    geoms_ = {front.contact_geom_id, rear.contact_geom_id};
    // :100-110 — every geom on each wheel body must be collision-
    // disabled for the pneumatic backend, wheels in insertion order.
    const std::span<const int> body_of = model_access::readonly_buffer(
            m_->geom_bodyid, m_->ngeom);
    const std::span<const int> contype = model_access::readonly_buffer(
            m_->geom_contype, m_->ngeom);
    const std::span<const int> conaffinity = model_access::readonly_buffer(
            m_->geom_conaffinity, m_->ngeom);
    for (const int body: bodies_)
        for (int g = 0; g < m_->ngeom; ++g)
            if (body_of[static_cast<std::size_t>(g)] == body &&
                (contype[static_cast<std::size_t>(g)] ||
                 conaffinity[static_cast<std::size_t>(g)]))
                throw std::invalid_argument(
                    "native wheel collisions must be disabled for "
                    "the pneumatic backend");
    // :111-117 — bodies/geoms/radii and the tier-derived wire shape.
    const std::span<const mjtNum> gsize = model_access::readonly_buffer(
            m_->geom_size, model_access::kXYZ * m_->ngeom);
    for (std::size_t i = 0; i < 2; ++i)
        radii_[i] = static_cast<double>(
                gsize[model_access::kXYZ *
                      static_cast<std::size_t>(geoms_[i])]);
    n_rays_ = pneu.n_rays;
    max_patches_ =
            (static_cast<std::size_t>(n_rays_) + 1) / 2;
    discretised_ = pneu.discretised_brush;
    const std::size_t n = static_cast<std::size_t>(n_rays_);
    const std::size_t nv = static_cast<std::size_t>(nv_);
    // The longest surface name the per-tick path can meet — the map's
    // default, the configured override and every section — sized once
    // so the patch-slot surface_name strings and the diagnostics'
    // surface field assign into capacity on the marked warm path.
    std::size_t surface_capacity = surface_map_.surface.name.size();
    if (pneu.surface.has_value())
        surface_capacity =
                std::max(surface_capacity, pneu.surface->name.size());
    for (const auto &sec: surface_map_.sections)
        surface_capacity =
                std::max(surface_capacity, sec.surface.name.size());
    // :118-121 — the velocity/Jacobian scratch and the live tyre pair —
    // plus the two clone banks the rollback and probe paths evaluate
    // through (all six are construction-sized; only the live models
    // prewarm, the clone banks' lazily-built engines are cold-path
    // allocation by design).
    jacp_.resize(model_access::kXYZ * nv);
    tmp_.resize(nv);
    qfrc_out_.resize(nv);
    hit_r_.resize(n);
    hit_delta_.resize(n);
    hit_rx_.resize(n);
    hit_rz_.resize(n);
    energy_scratch_.resize(n);
    shear_scratch_.resize(n);
    bristle_row_.resize(n);
    for (std::size_t s = 0; s < 2; ++s) {
        const nativecfg::PneumaticTyreSpec &spec =
                s == 0 ? pneu.front : pneu.rear;
        tyres_[s] = std::make_unique<biketyre::PneumaticTyreModel>(
                spec, n_rays_, pneu.half_angle_deg, discretised_);
        backup_[s] = std::make_unique<biketyre::PneumaticTyreModel>(
                spec, n_rays_, pneu.half_angle_deg, discretised_);
        probe_[s] = std::make_unique<biketyre::PneumaticTyreModel>(
                spec, n_rays_, pneu.half_angle_deg, discretised_);
        // Strictly allocation-free evaluate() on the warm path: every
        // brush width the migration could size exists up front, and
        // every patch slot's surface_name is reserved to the longest
        // name this writer's map can emit — the rollback/probe banks
        // get the same reserve because assign() and probe evaluate()
        // write those slots too.
        tyres_[s]->prewarm_detailed_brushes();
        for (biketyre::PneumaticTyreModel *model:
             {tyres_[s].get(), backup_[s].get(), probe_[s].get()})
            model->reserve_patch_names(surface_capacity);
        dec_delta_[s].resize(n);
        dec_elastic_[s].resize(n);
        dec_maxwell_[s].resize(n);
        dec_bristle_[s].resize(n);
        dec_covered_[s].resize(n);
        patch_stage_[s].resize(max_patches_);
        for (biketyre::PatchBrushState &slot: patch_stage_[s])
            slot.bristle_deflection_m.reserve(n);
        // Commit capacity — the marked warm path's assign-into-capacity
        // discipline: the patch bound plus the literal backend string
        // and the longest surface name the wire can carry.
        constexpr std::string_view kBackend = "pneumatic";
        for (auto *slot: {&snapshot_stage_[s], &snapshots_[s],
                          &probe_snapshots_[s]}) {
            slot->patches.reserve(max_patches_);
            slot->backend.reserve(kBackend.size());
        }
    }
    for (std::size_t s = 0; s < 2; ++s) {
        diagnostics_[s].surface.reserve(surface_capacity);
        diagnostics_stage_[s].surface.reserve(surface_capacity);
    }
    // :136-146 — the canonical wire columns, built once: side-major,
    // ray-major leaf order, then carcass/rim/patch records.
    const std::size_t side_cols = 4 * n + 8 + 3 * max_patches_;
    const std::size_t cols = 2 * side_cols;
    state_names_.reserve(cols);
    canonical_.reserve(cols);
    wire_cols_.resize(cols);
    wire_seen_.resize(cols);
    for (const char *side: kSides) {
        for (std::int64_t i = 0; i < n_rays_; ++i)
            for (const char *leaf: kRayLeaves) {
                std::string name = std::format("{}.ray.{:03d}.{}", side,
                                               i, leaf);
                canonical_.emplace(name, state_names_.size());
                state_names_.push_back(std::move(name));
            }
        {
            std::string name =
                    std::format("{}.carcass.initialized", side);
            canonical_.emplace(name, state_names_.size());
            state_names_.push_back(std::move(name));
        }
        for (const char *leaf: kRimLeaves) {
            std::string name = std::format("{}.rim.{}", side, leaf);
            canonical_.emplace(name, state_names_.size());
            state_names_.push_back(std::move(name));
        }
        {
            std::string name = std::format("{}.patch.count", side);
            canonical_.emplace(name, state_names_.size());
            state_names_.push_back(std::move(name));
        }
        for (std::size_t k = 0; k < max_patches_; ++k)
            for (const char *leaf: kPatchLeaves) {
                std::string name = std::format("{}.patch.{:02d}.{}", side,
                                               k, leaf);
                canonical_.emplace(name, state_names_.size());
                state_names_.push_back(std::move(name));
            }
    }
    // self.reset() (:122) — the member initializers hold the cleared
    // values already; the models were fresh-constructed.
}

void PneumaticTireWriter::reset() {
    // :124-130 — per-tyre reset, then the cleared artifacts and clock.
    // Copy-assign from const empties keeps the committed slots' reserved
    // capacity — like DistributedTireWriter::reset.
    for (const auto &tyre: tyres_)
        tyre->reset();
    static const TireSnapshot empty_snapshot;
    static const TireDiagnostics empty_diagnostics{};
    for (TireSnapshot &snap: snapshots_)
        snap = empty_snapshot;
    for (TireDiagnostics &diag: diagnostics_)
        diag = empty_diagnostics;
    committed_ = false;
    elastic_energy_j_ = 0.0;
    brush_loss_step_j_ = 0.0;
    radial_dissipation_power_w_ = 0.0;
    last_time_s_.reset();
}

const std::vector<std::string> &
PneumaticTireWriter::state_names() const {
    return state_names_;
}

std::vector<double> PneumaticTireWriter::state() const {
    // :148-177 — one row of 2 * (4*n + 8 + 3*width) doubles, NaN for
    // unset bristle cells, inactive rim fields and unused patch slots.
    const std::size_t n = static_cast<std::size_t>(n_rays_);
    const std::size_t width = max_patches_;
    const std::size_t side_cols = 4 * n + 8 + 3 * width;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> row(2 * side_cols, nan);
    for (std::size_t s = 0; s < 2; ++s) {
        const biketyre::PneumaticTyreModel &tyre = *tyres_[s];
        const std::size_t base = s * side_cols;
        // bristle = np.full(n, nan); patch bristle j → ray start+j.
        std::ranges::fill(bristle_row_, nan);
        for (const biketyre::PatchBrushState &state:
             tyre.patch_states()) {
            if (!state.has_detailed)
                continue;
            // `bristle[start:stop] = row` — the NumPy slice-assign's
            // width-mismatch ValueError lands as invalid_argument.
            const std::size_t want =
                    static_cast<std::size_t>(state.stop - state.start);
            if (state.bristle_deflection_m.size() != want)
                throw std::invalid_argument(
                    "pneumatic bristle row could not broadcast onto the "
                    "ray cells");
            std::ranges::copy(
                    state.bristle_deflection_m,
                    bristle_row_.begin() +
                            static_cast<std::ptrdiff_t>(state.start));
        }
        const biketyre::CarcassState &carcass = tyre.carcass_state();
        for (std::size_t i = 0; i < n; ++i) {
            row[base + 4 * i + 0] = carcass.delta_prev_m()[i];
            row[base + 4 * i + 1] = carcass.elastic_prev_n()[i];
            row[base + 4 * i + 2] = carcass.maxwell_force_n()[i];
            row[base + 4 * i + 3] = bristle_row_[i];
        }
        row[base + 4 * n] = carcass.initialized() ? 1.0 : 0.0;
        const std::size_t rim = base + 4 * n + 1;
        row[rim] = carcass.rim_strike_active() ? 1.0 : 0.0;
        if (carcass.rim_strike_active()) {
            row[rim + 1] = carcass.rim_event_x_m();
            row[rim + 2] = carcass.rim_event_speed_mps();
            row[rim + 3] = carcass.rim_event_peak_load_n();
            row[rim + 4] = carcass.rim_event_peak_force_n();
            row[rim + 5] = carcass.rim_event_energy_j();
        }
        const std::span<const biketyre::PatchBrushState> patches =
                tyre.patch_states();
        row[rim + 6] = static_cast<double>(patches.size());
        for (std::size_t k = 0; k < patches.size(); ++k) {
            row[rim + 7 + 3 * k] =
                    static_cast<double>(patches[k].start);
            row[rim + 7 + 3 * k + 1] =
                    static_cast<double>(patches[k].stop);
            row[rim + 7 + 3 * k + 2] = patches[k].kappa_prime;
        }
    }
    return row;
}

void PneumaticTireWriter::set_state(
        std::span<const std::string> names, std::span<const double> row) {
    // :179-293 — the strict canonical-row contract documented in the
    // header: unknown columns first (first-occurrence order), then the
    // equal-length gate, canonical coverage, the two-phase decode, then
    // the sequential per-side commit and the clock re-arm.
    std::ranges::fill(wire_seen_, static_cast<char>(0));
    for (const std::string &name: names) {
        const auto it = canonical_.find(name);
        if (it == canonical_.end())
            throw std::runtime_error("tire state has unknown column " +
                                     name);
        if (wire_seen_[it->second] != 0)
            throw std::runtime_error(
                    "tire state has duplicate column " + name);
        wire_seen_[it->second] = 1;
    }
    if (names.size() != row.size())
        throw std::runtime_error("tire state row has (" +
                                 std::to_string(row.size()) +
                                 ",) cells for " +
                                 std::to_string(names.size()) +
                                 " columns");
    for (std::size_t c = 0; c < state_names_.size(); ++c)
        if (wire_seen_[c] == 0)
            throw std::runtime_error("tire state is missing column " +
                                     state_names_[c]);
    // `index` — the oracle's {name: i} map gathers the same values.
    for (std::size_t i = 0; i < names.size(); ++i)
        wire_cols_[canonical_.find(names[i])->second] = row[i];
    const std::size_t n = static_cast<std::size_t>(n_rays_);
    const std::size_t width = max_patches_;
    const std::size_t side_cols = 4 * n + 8 + 3 * width;
    // Per-side scalar fields staged beside the decoded rows.
    struct DecodedScalars {
        bool initialized = false, rim_active = false;
        std::size_t patch_count = 0;
    };
    std::array<DecodedScalars, 2> decoded;
    for (std::size_t s = 0; s < 2; ++s) {
        const std::size_t base = s * side_cols;
        for (std::size_t i = 0; i < n; ++i) {
            dec_delta_[s][i] = wire_cols_[base + 4 * i];
            dec_elastic_[s][i] = wire_cols_[base + 4 * i + 1];
            dec_maxwell_[s][i] = wire_cols_[base + 4 * i + 2];
            dec_bristle_[s][i] = wire_cols_[base + 4 * i + 3];
        }
        for (const auto &[values, leaf]:
             std::array<std::pair<const std::vector<double> *,
                                  const char *>,
                        3>{{{&dec_delta_[s], "delta_prev_m"},
                           {&dec_elastic_[s], "elastic_prev_n"},
                           {&dec_maxwell_[s], "maxwell_n"}}})
            if (!std::ranges::all_of(*values, [](double v) {
                    return std::isfinite(v);
                }))
                throw std::runtime_error(
                        std::string("tire state ray ") + leaf +
                        " cells must be finite");
        const double initialized = wire_cols_[base + 4 * n];
        if (initialized != 0.0 && initialized != 1.0)
            throw std::runtime_error(
                    std::string("tire state column ") + kSides[s] +
                    ".carcass.initialized must be 0 or 1");
        const double rim_active = wire_cols_[base + 4 * n + 1];
        if (rim_active != 0.0 && rim_active != 1.0)
            throw std::runtime_error(
                    std::string("tire state column ") + kSides[s] +
                    ".rim.active must be 0 or 1");
        const std::size_t rim = base + 4 * n + 1;
        bool finite_rim_any = false, finite_rim_all = true;
        for (std::size_t k = 1; k < kRimLeaves.size(); ++k) {
            const bool f = std::isfinite(wire_cols_[rim + k]);
            finite_rim_any |= f;
            finite_rim_all &= f;
        }
        if (rim_active == 1.0 && !finite_rim_all)
            throw std::runtime_error(
                    std::string("tire state ") + kSides[s] +
                    ".rim fields must be finite while active");
        if (rim_active == 0.0 && finite_rim_any)
            throw std::runtime_error(
                    std::string("tire state ") + kSides[s] +
                    ".rim fields must be NaN while inactive");
        const double count_value = wire_cols_[rim + 6];
        // `count_value != int(count_value)` — a nonnegative finite cell
        // integral under truncation.
        if (!std::isfinite(count_value) || count_value < 0.0 ||
            count_value != std::trunc(count_value))
            throw std::runtime_error(
                    std::string("tire state column ") + kSides[s] +
                    ".patch.count must be a nonnegative integer");
        // The oracle's `int(count_value)` accepts arbitrary magnitude
        // before `count > width` rejects — compare in the double domain
        // so the cast below never leaves [0, width].
        if (count_value > static_cast<double>(width))
            throw std::runtime_error(
                    std::string("tire state column ") + kSides[s] +
                    ".patch.count exceeds " + std::to_string(width));
        const std::size_t count = static_cast<std::size_t>(count_value);
        std::ranges::fill(dec_covered_[s], static_cast<char>(0));
        double previous_stop = -1.0;
        for (std::size_t k = 0; k < count; ++k) {
            const double start = wire_cols_[rim + 7 + 3 * k];
            const double stop = wire_cols_[rim + 7 + 3 * k + 1];
            const double kappa_prime = wire_cols_[rim + 7 + 3 * k + 2];
            for (const auto &[value, leaf]:
                 std::array<std::pair<double, const char *>, 3>{{
                     {start, "start"},
                     {stop, "stop"},
                     {kappa_prime, "kappa_prime"}}})
                if (!std::isfinite(value))
                    throw std::runtime_error(std::format(
                            "tire state {}.patch.{:02d}.{} must be "
                            "finite",
                            kSides[s], k, leaf));
            if (start != std::trunc(start) || stop != std::trunc(stop))
                throw std::runtime_error(std::format(
                        "tire state {}.patch.{:02d} bounds must be "
                        "integers",
                        kSides[s], k));
            // `0 <= start < stop <= n` on integral doubles is the int
            // comparison — evaluate it before the casts keep si/ei in
            // [0, n_rays].
            if (!(start >= 0.0 && start < stop &&
                  stop <= static_cast<double>(n_rays_)))
                throw std::runtime_error(std::format(
                        "tire state {}.patch.{:02d} bounds outside the "
                        "ray ring",
                        kSides[s], k));
            if (start < previous_stop)
                throw std::runtime_error(
                        std::string("tire state ") + kSides[s] +
                        " patches must be disjoint in ray order");
            previous_stop = stop;
            const int si = static_cast<int>(start);
            const int ei = static_cast<int>(stop);
            for (int i = si; i < ei; ++i)
                dec_covered_[s][static_cast<std::size_t>(i)] = 1;
            biketyre::PatchBrushState &slot = patch_stage_[s][k];
            slot.start = si;
            slot.stop = ei;
            slot.kappa_prime = kappa_prime;
            slot.has_detailed = discretised_;
            slot.detailed_kappa_prime = discretised_ ? kappa_prime : 0.0;
            if (discretised_)
                slot.bristle_deflection_m.assign(
                        dec_bristle_[s].begin() + si,
                        dec_bristle_[s].begin() + ei);
            else
                slot.bristle_deflection_m.clear();
        }
        for (std::size_t k = count; k < width; ++k)
            for (std::size_t leaf = 0; leaf < kPatchLeaves.size(); ++leaf)
                if (std::isfinite(wire_cols_[rim + 7 + 3 * k + leaf]))
                    throw std::runtime_error(std::format(
                            "tire state {}.patch.{:02d} is set past "
                            "patch.count",
                            kSides[s], k));
        if (discretised_) {
            for (std::size_t i = 0; i < n; ++i) {
                if (dec_covered_[s][i] != 0 &&
                    !std::isfinite(dec_bristle_[s][i]))
                    throw std::runtime_error(
                            std::string("tire state ") + kSides[s] +
                            " patch rays need finite bristle cells");
                if (dec_covered_[s][i] == 0 &&
                    std::isfinite(dec_bristle_[s][i]))
                    throw std::runtime_error(
                            std::string("tire state ") + kSides[s] +
                            " bristle cells are set outside every "
                            "patch");
            }
        } else {
            for (std::size_t i = 0; i < n; ++i)
                if (std::isfinite(dec_bristle_[s][i]))
                    throw std::runtime_error(
                            std::string("tire state ") + kSides[s] +
                            " bristle cells must be NaN on the lumped "
                            "tier");
        }
        decoded[s] = {.initialized = initialized == 1.0,
                      .rim_active = rim_active == 1.0,
                      .patch_count = count};
    }
    // :267-292 — commit, side-sequential like the oracle: the decode is
    // fully validated above so restore_patch_states cannot reject here.
    for (std::size_t s = 0; s < 2; ++s) {
        biketyre::PneumaticTyreModel &tyre = *tyres_[s];
        const std::size_t base = s * side_cols;
        const std::size_t rim = base + 4 * n + 1;
        tyre.reset();
        biketyre::CarcassState &carcass = tyre.carcass_state();
        std::ranges::copy(dec_delta_[s],
                          carcass.delta_prev_m().begin());
        std::ranges::copy(dec_elastic_[s],
                          carcass.elastic_prev_n().begin());
        std::ranges::copy(dec_maxwell_[s],
                          carcass.maxwell_force_n().begin());
        if (decoded[s].rim_active)
            carcass.restore_fields(
                    decoded[s].initialized, true,
                    wire_cols_[rim + 1], wire_cols_[rim + 2],
                    wire_cols_[rim + 3], wire_cols_[rim + 4],
                    wire_cols_[rim + 5]);
        else
            carcass.restore_fields(decoded[s].initialized, false, 0.0,
                                   0.0, 0.0, 0.0, 0.0);
        tyre.restore_patch_states(std::span<const biketyre::PatchBrushState>(
                                          patch_stage_[s])
                                          .first(decoded[s].patch_count));
    }
    last_time_s_.reset();
}

// _intersect (pneumatic_tire_forces.py:295-303) into the shared
// hit-buffer scratch — caller reads `out` before the next call.
void PneumaticTireWriter::intersect_at(
        const biketyre::PneumaticTyreModel &tyre, double hub_x_m,
        double hub_z_m, biketyre::RayHitBuffers &out) const {
    out = biketyre::RayHitBuffers{.r_m = hit_r_,
                                 .delta_m = hit_delta_,
                                 .road_x_m = hit_rx_,
                                 .road_z_m = hit_rz_};
    biketyre::intersect_into(tyre.ring(), hub_x_m, hub_z_m,
                             tyre.tyre().outer_radius_mm / 1000.0,
                             engaged(profile_), out);
}

// _radial_energy_j (pneumatic_tire_forces.py:57-66): elementwise
// 0.5*k_c*δ²*ds + 0.5*k_rim*overlap²*ds folded by np.sum.
double PneumaticTireWriter::radial_energy_j(
        const biketyre::PneumaticTyreModel &tyre,
        std::span<const double> delta_m) const {
    const nativecfg::PneumaticTyreSpec &spec = tyre.tyre();
    const double ds_m =
            spec.outer_radius_mm * 0.001 * tyre.ring().dtheta_rad();
    const double k_c = spec.carcass_stiffness_n_mm2 * 1000000.0;
    const double k_rim = spec.rim_stiffness_n_mm2 * 1000000.0;
    const double rim_m = spec.rim_strike_deflection_mm() * 0.001;
    for (std::size_t i = 0; i < delta_m.size(); ++i) {
        const double overlap =
                numeric::np_maximum(delta_m[i] - rim_m, 0.0);
        energy_scratch_[i] =
                0.5 * k_c * delta_m[i] * delta_m[i] * ds_m +
                0.5 * k_rim * overlap * overlap * ds_m;
    }
    return numeric::numpy_pairwise_sum(
            std::span<const double>(energy_scratch_)
                    .first(delta_m.size()));
}

// _shear_energy_j (pneumatic_tire_forces.py:305-330): committed-patch
// bristle energy on the detailed tier; the outputs dict keyed by
// (ray_start, ray_stop) misses unset ranges exactly like the oracle's
// .get() returning None.
double PneumaticTireWriter::shear_energy_j(
        const biketyre::PneumaticTyreModel &tyre) const {
    if (!discretised_)
        return 0.0;
    double energy = 0.0;
    for (const biketyre::PatchBrushState &state: tyre.patch_states()) {
        if (!state.has_detailed)
            continue;
        // `outputs.get((start, stop))` — dict comprehension last-write-
        // wins on a duplicated key: take the LAST matching patch.
        const biketyre::TyrePatchOutput *patch = nullptr;
        for (const biketyre::TyrePatchOutput &p:
             tyre.last_outputs().patches())
            if (p.ray_start == state.start && p.ray_stop == state.stop)
                patch = &p;
        if (patch == nullptr)
            continue;
        const double half_length_m = 0.5 * patch->contact_length_m;
        if (!(half_length_m > 0.0))
            continue;
        const nativecfg::SurfaceSpec &surface = tirecommon::surface_at(
                surface_map_, patch->centroid_world_m[0]);
        const double k_px =
                surface.slip_stiffness_per_load * patch->normal_load_n /
                (2.0 * pyfloat::float_pow(half_length_m, 2.0));
        const std::size_t elements = state.bristle_deflection_m.size();
        const double dx_m =
                2.0 * half_length_m / static_cast<double>(elements);
        for (std::size_t i = 0; i < elements; ++i)
            shear_scratch_[i] = state.bristle_deflection_m[i] *
                                state.bristle_deflection_m[i];
        energy += 0.5 * k_px * dx_m *
                  numeric::numpy_pairwise_sum(
                          std::span<const double>(shear_scratch_)
                                  .first(elements));
    }
    return energy;
}

// stored_energy (pneumatic_tire_forces.py:332-346): re-intersect at the
// CURRENT pose, then radial + shear folds in front/rear order.
double PneumaticTireWriter::stored_energy(const mjData *d) const {
    if (d == nullptr)
        throw std::invalid_argument(
            "tire stored_energy needs a live mjData");
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
            d->xpos, model_access::kXYZ * m_->nbody);
    double energy = 0.0;
    for (std::size_t s = 0; s < 2; ++s) {
        const biketyre::PneumaticTyreModel &tyre = *tyres_[s];
        const std::size_t b3 =
                model_access::kXYZ * static_cast<std::size_t>(bodies_[s]);
        biketyre::RayHitBuffers hits;
        intersect_at(tyre, xpos[b3], xpos[b3 + 2], hits);
        energy += radial_energy_j(
                tyre, std::span<const double>(hits.delta_m));
        energy += shear_energy_j(tyre);
    }
    return energy;
}

// Caller precondition: `d` is the live mjData of the model this writer
// was built on — the single owning context's contract.
void PneumaticTireWriter::compute_into(const mjData *d, double dt,
                                       std::span<double> out) {
    if (d == nullptr)
        throw std::invalid_argument("tire compute needs a live mjData");
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()))
        throw std::invalid_argument("tire output width");
    accumulate_validated(d, dt, out);
}

// The warm-core twin of compute_into: precondition gates report
// CoreStatus::invalid_input and residual kernel rejections are caught
// and mapped so noexcept is proved by the catch-all. A valid call never
// allocates — models prewarm every brush width at construction and
// every scratch row is construction-sized.
CoreStatus PneumaticTireWriter::try_compute_into(
        const mjData *d, double dt, std::span<double> out) noexcept
BIKE_NONBLOCKING {
    if (d == nullptr || out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()) || !std::isfinite(dt) ||
        dt <= 0.0 ||
        (last_time_s_.has_value() &&
         static_cast<double>(d->time) <= *last_time_s_))
        return CoreStatus::invalid_input;
    try {
        accumulate_validated(d, dt, out);
    } catch (const std::bad_alloc &) {
        return CoreStatus::engine_failure;
    } catch (const engine::EngineFailure &) {
        return CoreStatus::engine_failure;
    } catch (const std::exception &) {
        return CoreStatus::invalid_input;
    } catch (...) {
        return CoreStatus::engine_failure;
    }
    return CoreStatus::ok;
}

std::vector<double> PneumaticTireWriter::qfrc(const mjData *d,
                                              double dt) {
    compute_into(d, dt, qfrc_out_);
    return qfrc_out_;
}

// compute_qfrc(advance=True) (pneumatic_tire_forces.py:356-475). The
// interval scalar gate and the once-per-timestamp gate precede the
// backup; the whole two-wheel fold runs inside the try so a throw
// anywhere restores the pre-loop semantic state before propagating.
// The committed slots copy-assign from the stages — capacity-reserved
// at construction.
void PneumaticTireWriter::accumulate_validated(const mjData *d,
                                               double dt_arg,
                                               std::span<double> out) {
    // scalar(dt, 'pneumatic tire interval', positive=True) (:356) and
    // the double-advance gate (:357-359).
    if (!std::isfinite(dt_arg) || dt_arg <= 0.0)
        throw std::invalid_argument("invalid pneumatic tire interval");
    const double dt = dt_arg;
    const double time = static_cast<double>(d->time);
    if (last_time_s_.has_value() && time <= *last_time_s_)
        throw std::invalid_argument(
            "tire state can advance only once per increasing timestamp");
    // :360-366 — zeros(nv) result happens inside evaluate_wheels; the
    // pre-loop deepcopy lands as the semantic assign() into the
    // construction-sized backup bank.
    backup_[0]->assign(*tyres_[0]);
    backup_[1]->assign(*tyres_[1]);
    try {
        double energy = 0.0, brush_loss = 0.0, radial_power = 0.0;
        evaluate_wheels(d, dt, time, out, tyres_, energy, brush_loss,
                        radial_power);
        // :471-474 — the atomic commit after both wheels evaluate.
        snapshots_[0] = snapshot_stage_[0];
        snapshots_[1] = snapshot_stage_[1];
        diagnostics_[0] = diagnostics_stage_[0];
        diagnostics_[1] = diagnostics_stage_[1];
        committed_ = true;
        elastic_energy_j_ = energy;
        brush_loss_step_j_ = brush_loss;
        radial_dissipation_power_w_ = radial_power;
        last_time_s_ = time;
    } catch (...) {
        // :468-470 — `self.states = backup`: restore BOTH models'
        // semantic state before the error propagates.
        tyres_[0]->assign(*backup_[0]);
        tyres_[1]->assign(*backup_[1]);
        throw;
    }
}

// The oracle's wheel loop (:368-463) plus the tail derived_* gates
// (:464-467), parameterized over the model bank so the probe path runs
// the same sequence on its clones. Snapshot and diagnostic rows stage
// into the members the caller commits/publishes.
// NOLINTBEGIN(bugprone-easily-swappable-parameters) out-params mirror the oracle's three accumulators
void PneumaticTireWriter::evaluate_wheels(
        const mjData *d, double dt, double time, std::span<double> out,
        const std::array<std::unique_ptr<biketyre::PneumaticTyreModel>,
                         2> &tyres,
        double &energy_out, double &brush_loss_out,
        double &radial_power_out) {
    // NOLINTEND(bugprone-easily-swappable-parameters)
    std::ranges::fill(out, 0.0); // :360 — np.zeros(model.nv)
    const std::size_t nv = static_cast<std::size_t>(nv_);
    const nativecfg::PneumaticConfig &pneu = engaged(cfg_.pneumatic);
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
            d->xpos, model_access::kXYZ * m_->nbody);
    double energy = 0.0, brush_loss = 0.0, radial_power = 0.0;
    static const TireDiagnostics empty_diagnostics{};
    // :368 — self.bodies.items() — front then rear, insertion order.
    for (std::size_t s = 0; s < 2; ++s) {
        const int body = bodies_[s];
        biketyre::PneumaticTyreModel &tyre = *tyres[s];
        // :371-378 — mj_objectVelocity into the zeroed velocity6, the
        // hub row and the advancing whole-wheel evaluate.
        velocity6_.fill(0.0);
        mj_objectVelocity(m_, d, mjOBJ_BODY, body, velocity6_.data(), 0);
        const std::size_t b3 =
                model_access::kXYZ * static_cast<std::size_t>(body);
        const Vec3 hub = {static_cast<double>(xpos[b3]),
                          static_cast<double>(xpos[b3 + 1]),
                          static_cast<double>(xpos[b3 + 2])};
        const biketyre::WheelOutputs &outputs = tyre.evaluate(
                std::span<const double>(hub),
                std::span<const mjtNum>(velocity6_).subspan(
                        model_access::kXYZ, model_access::kXYZ),
                static_cast<double>(velocity6_[1]), engaged(profile_),
                surface_map_, dt);
        // :383-386 — every patch force maps through mj_jac at ITS
        // centroid, sequentially: result += jacp.T @ patch.force. The
        // ContactPatch pass (:387-398) is a second loop like the
        // oracle's — its post_init gates never fire while the force
        // fold is still running.
        for (const biketyre::TyrePatchOutput &patch: outputs.patches()) {
            model_access::point_jacobian_into(
                    m_, d, body,
                    std::span<const mjtNum>(patch.centroid_world_m),
                    std::span<mjtNum>(jacp_),
                    model_access::JacobianBody::physical_strict,
                    "tire patch centroid");
            blas::dgemv(blas::Order::row_major, blas::Transpose::yes,
                        model_access::kXYZ, nv_, 1.0, jacp_.data(), nv_,
                        patch.force_world_n.data(), 1, 0.0, tmp_.data(),
                        1);
            for (std::size_t i = 0; i < nv; ++i)
                out[i] += tmp_[i];
        }
        TireSnapshot &snap = snapshot_stage_[s];
        snap.patches.clear();
        for (const biketyre::TyrePatchOutput &patch: outputs.patches()) {
            // :390-397 — the ContactPatch row: normal is the patch's
            // normal FORCE divided by its load — the oracle's
            // elementwise divide, not the model's normalized vector.
            const double load = patch.normal_load_n;
            const Vec3 normal = {
                    patch.normal_force_world_n[0] / load,
                    patch.normal_force_world_n[1] / load,
                    patch.normal_force_world_n[2] / load};
            // Construction-reserved patch capacity — the bound can
            // never trip on a valid evaluation.
            if (snap.patches.size() >= max_patches_)
                throw std::invalid_argument(
                        "tire snapshot patch capacity exceeded");
            snap.patches.push_back(make_patch(
                    patch.centroid_world_m, normal, load,
                    patch.tangential_force_n, patch.slip_speed_mps));
        }
        // :399-403 — WheelContactSnapshot(...): arguments evaluate in
        // order — interval_id(time, dt) raises before __post_init__ —
        // then the __post_init__ gates in their own order
        // (contact_state.py:95-113): time, interval ID, backend,
        // patch types (unreachable for real rows), axis.
        snap.time_s = time;
        snap.geometric_contact = !outputs.airborne;
        snap.interval_id = interval_clock::interval_id(time, dt);
        snap.backend = "pneumatic";
        snap.wheel_axis_m = hub;
        if (!std::isfinite(time) || time < 0.0)
            throw std::invalid_argument(
                    "contact snapshot needs finite nonnegative time");
        if (snap.interval_id < 0)
            throw std::invalid_argument(
                    "contact snapshot needs a nonnegative interval ID");
        for (const double v: hub)
            if (!std::isfinite(v))
                throw std::invalid_argument(
                        "wheel axis must be a finite 3D vector");
        snap.effective_radius_m =
                effective_radius(snap.patches, hub);
        // :404-409 — a fresh intersect at the same hub feeds the
        // radial-energy and penetration channels; the shear fold reads
        // the committed brush state.
        biketyre::RayHitBuffers hits;
        intersect_at(tyre, hub[0], hub[2], hits);
        const double radial_energy =
                radial_energy_j(tyre, std::span<const double>(
                                              hits.delta_m));
        const double shear_energy = shear_energy_j(tyre);
        const std::size_t patch_count = outputs.patch_count;
        numeric::PythonSum brush_power;
        for (const biketyre::TyrePatchOutput &patch: outputs.patches())
            brush_power.add(patch.dissipated_power_w);
        const double brush_power_w = brush_power.total();
        const double load = outputs.normal_load_n;
        // :407-420 — mu is assigned on both branches (load-weighted mean
        // patch mu, else the Stribeck fallback at the hub's surface);
        // the 0.0 seed is never observed.
        double slip_mps = 0.0, mu = 0.0;
        if (load > 0.0) {
            numeric::PythonSum slip_num, mu_num;
            for (const biketyre::TyrePatchOutput &patch:
                 outputs.patches()) {
                slip_num.add(patch.slip_speed_mps *
                             patch.normal_load_n);
                mu_num.add(patch.friction_coefficient *
                           patch.normal_load_n);
            }
            slip_mps = slip_num.total() / load;
            mu = mu_num.total() / load;
        } else {
            // :418 — surface_map.at(hub_x).mu(0.0): the Stribeck law.
            mu = tirecommon::surface_mu(
                    tirecommon::surface_at(surface_map_, hub[0]), 0.0);
        }
        // :422-424 — the patch's own surface name, else the configured
        // surface, else the track lookup at hub x.
        const std::string_view surface_name =
                patch_count > 0
                        ? std::string_view(
                                  outputs.patches()[0].surface_name)
                        : (pneu.surface.has_value()
                                   ? std::string_view(pneu.surface->name)
                                   : std::string_view(
                                             tirecommon::surface_at(
                                                     surface_map_,
                                                     hub[0])
                                                     .name));
        // :425-456 — diagnostics[side], one fresh dict per side per
        // tick (the empty-assign keeps staged string capacity).
        TireDiagnostics &diag = diagnostics_stage_[s];
        diag = empty_diagnostics;
        diag.normal_load_n = load;
        diag.support_n = outputs.support_n;
        {
            numeric::PythonSum tangent_total;
            for (const biketyre::TyrePatchOutput &patch:
                 outputs.patches())
                tangent_total.add(patch.tangential_force_n);
            diag.tangent_force_n = tangent_total.total();
        }
        diag.slip_mps = slip_mps;
        diag.slip_ratio = outputs.slip_ratio;
        diag.transient_slip_ratio = outputs.transient_slip_ratio;
        diag.friction_coefficient = mu;
        // float(np.max(hits.delta_m)) — sequential np_maximum fold.
        double penetration = -std::numeric_limits<double>::infinity();
        for (const double v: std::span<const double>(hits.delta_m))
            penetration = numeric::np_maximum(penetration, v);
        diag.penetration_m = penetration;
        diag.contact_length_m = outputs.contact_length_m;
        diag.patch_count = static_cast<int>(patch_count);
        diag.multi_support = patch_count > 1;
        diag.supports_multiple_contacts = true;
        diag.fully_sliding = outputs.fully_sliding;
        diag.airborne = outputs.airborne;
        diag.coverage_event = outputs.coverage_event;
        diag.rim_strike_active = outputs.rim_strike_active;
        diag.rim_event_x_m = outputs.rim_event.has_value()
                                     ? outputs.rim_event->x_m : 0.0;
        diag.rim_event_speed_mps =
                outputs.rim_event.has_value()
                        ? outputs.rim_event->speed_mps : 0.0;
        diag.rim_event_peak_load_n =
                outputs.rim_event.has_value()
                        ? outputs.rim_event->peak_load_n : 0.0;
        diag.rim_event_peak_force_n =
                outputs.rim_event.has_value()
                        ? outputs.rim_event->peak_rim_force_n : 0.0;
        diag.rim_event_energy_j =
                outputs.rim_event.has_value()
                        ? outputs.rim_event->absorbed_energy_j : 0.0;
        diag.pressure_bar = outputs.pressure_bar;
        diag.dissipated_power_w = outputs.dissipated_power_w;
        diag.brush_loss_j = brush_power_w * dt;
        diag.radial_energy_j = radial_energy;
        diag.shear_energy_j = shear_energy;
        diag.outside_material_load_range = false;
        diag.surface = surface_name;
        // :460-463 — the cross-side folds, sequential adds like
        // stored_energy's own pair.
        energy += radial_energy;
        energy += shear_energy;
        brush_loss += brush_power_w * dt;
        radial_power += outputs.dissipated_power_w - brush_power_w;
    }
    // :464-467 — the derived gates, inside the caller's try so a
    // non-finite fold still restores both models' state.
    if (!std::ranges::all_of(out,
                             [](double v) { return std::isfinite(v); }))
        throw std::overflow_error(
                "PneumaticTireForceApplier.qfrc: non-finite derived "
                "array");
    if (!std::isfinite(energy))
        throw std::overflow_error(
                "PneumaticTireForceApplier.energy: non-finite derived "
                "value");
    if (!std::isfinite(brush_loss))
        throw std::overflow_error(
                "PneumaticTireForceApplier.brush_loss: non-finite "
                "derived value");
    if (!std::isfinite(radial_power))
        throw std::overflow_error(
                "PneumaticTireForceApplier.radial_power: non-finite "
                "derived value");
    energy_out = energy;
    brush_loss_out = brush_loss;
    radial_power_out = radial_power;
}

// compute_qfrc(advance=False) (:349-355): the detached copy evaluates
// the advancing kernel on a cleared clock; only its committed snapshots
// cross back. The probe bank re-assigns the live models' semantic state
// each call — assign() is the deepcopy — so a thrown probe leaves the
// writer's committed state, clock and prior probe publication intact.
void PneumaticTireWriter::probe_compute_into(const mjData *d, double dt,
                                             std::span<double> out) {
    if (d == nullptr)
        throw std::invalid_argument("tire probe needs a live mjData");
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()))
        throw std::invalid_argument("tire output width");
    // The probe copy's compute_qfrc re-runs the interval scalar gate
    // before any work — its cleared clock is the only difference.
    if (!std::isfinite(dt) || dt <= 0.0)
        throw std::invalid_argument("invalid pneumatic tire interval");
    probe_[0]->assign(*tyres_[0]);
    probe_[1]->assign(*tyres_[1]);
    double energy = 0.0, brush_loss = 0.0, radial_power = 0.0;
    evaluate_wheels(d, dt, static_cast<double>(d->time), out, probe_,
                    energy, brush_loss, radial_power);
    probe_snapshots_ = snapshot_stage_;
    probe_committed_ = true;
}

std::vector<double>
PneumaticTireWriter::probe_qfrc(const mjData *d, double dt) {
    std::vector<double> out(static_cast<std::size_t>(nv_));
    probe_compute_into(d, dt, out);
    return out;
}
