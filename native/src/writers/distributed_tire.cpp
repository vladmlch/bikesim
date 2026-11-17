// writers/distributed_tire.cpp — port of distributed_tire_forces.py:
// construction, the canonical per-station state wire, stored energy, the
// reset/clock/snapshot/diagnostic scaffolding, and the advancing
// per-station tick (plan Tasks 4-5). Comments cite file:line at porting
// time.
//
// The same-libcall argument from writers/tire.cpp applies to `local @
// rotation.T`: numpy lowers the (N,3) @ (3,3) product to ONE
// cblas_dgemm — verified bitwise against Accelerate (0 of 1536 elements
// differ; a per-row dgemv/ddot loop differs on ~40%). The port invokes the
// same entry point through blas::dgemm, then adds `center` elementwise —
// the oracle evaluates the matmul fully before the broadcast add.
#include "distributed_tire.hpp"

#include "../cblas_abi.hpp"
#include "../config_validation.hpp"
#include "../diag.hpp"
#include "../engaged.hpp"
#include "../engine_call.hpp"
#include "../interval_clock.hpp"
#include "../model_access.hpp"
#include "../numeric_sincos.hpp"
#include "../numeric_sum.hpp"
#include "../physics_core/cpu_exact_toolkit.hpp"
#include "../physics_core/tyre_distributed.hpp"
#include "../tyre/profile.hpp"
#include "pyfloat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <format>
#include <limits>
#include <new>
#include <numbers>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
    using Vec3 = std::array<double, model_access::kXYZ>;

    // physical_mapping.py:6-11 — resolve_id. Same plain name check
    // TireWriter applies to the wheel contact geoms.
    int resolve_id(const mjModel *m, mjtObj kind, const char *name) {
        return model_access::resolve_id(m, kind, name);
    }

    // physical_mapping.py:14-21 — point_jacobian: BOTH mj_jac halves at
    // one world point. The oracle gates on `array(point, 'world point',
    // (3,))` (finite triple — the Vec3 type carries the shape) and
    // `0 < body_id < model.nbody` — physical_mapping's single
    // physical_strict check — then fills two (3,nv) row-major outputs.
    // The construction-sized destinations re-prove their extents so the
    // engine ABI only ever sees validated buffers.
    void point_jacobian(const mjModel *m, const mjData *d, int body_id,
                        const Vec3 &point, std::span<mjtNum> jp,
                        std::span<mjtNum> jr) {
        if (!std::ranges::all_of(
                point, [](mjtNum v) { return std::isfinite(v); }))
            throw std::invalid_argument("invalid world point");
        if (!(0 < body_id && body_id < m->nbody))
            throw std::invalid_argument(
                "point Jacobian requires a physical body");
        if (m->nv < 0)
            throw std::invalid_argument(
                "point Jacobian needs a nonnegative model width");
        const std::size_t want = model_access::checked_product(
                static_cast<std::size_t>(model_access::kXYZ),
                static_cast<std::size_t>(m->nv),
                std::numeric_limits<std::size_t>::max());
        if (jp.size() != want || jr.size() != want)
            throw std::invalid_argument(
                "point Jacobian destination must be 3*nv elements");
        mj_jac(m, d, jp.data(), jr.data(), point.data(), body_id);
    }

    // np.linalg.norm(v) on a (3,) — sqrt(ddot(v,v)), the same-libcall
    // form. Mirrors the tire.cpp helper verbatim: the patch-validation
    // site's oracle is numpy's BLAS path, so it stays on ddot by design.
    double norm3(const Vec3 &v) {
        return std::sqrt(blas::ddot(model_access::kXYZ, v.data(), 1,
                                    v.data(), 1));
    }

    double dot3(const Vec3 &a, const Vec3 &b) {
        return blas::ddot(model_access::kXYZ, a.data(), 1, b.data(), 1);
    }

    // ContactPatch.__post_init__ (contact_state.py:32-59) — the fields
    // the distributed writer supplies: source 'terrain', no couple/
    // native force. Verbatim copy of the tire.cpp helper — identical
    // expressions, so identical accept/reject behaviour.
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
            .working_surface = true,
            .native_world_force_n = std::nullopt,
            .couple_world_nm = {}
        };
    }

    NATIVE_DIAG_POP

    // WheelContactSnapshot.effective_radius_m (contact_state.py:172-181):
    // the load-weighted mean contact drop; unit weights when every patch
    // is unloaded; 0 when there is no patch. numeric::PythonSum is the
    // builtin sum() Neumaier loop — identical operation order to the
    // tire.cpp helper's accumulator. Every item is a float patch field,
    // the 1.0 literal, or a float product — the compensated float branch
    // is the oracle's whole path.
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

    // distributed_tire_forces.py:32 — _STATION_LEAVES, in wire order.
    constexpr std::array<const char *, 4> kLeaves = {
        "xi", "tangent.0", "tangent.1", "tangent.2"
    };
    constexpr std::array<const char *, 2> kSides = {"front", "rear"};
} // namespace

// Caller preconditions (documented at the raw boundary): `d` must be an
// mjData created from `m` — dimension equality is a width check, never a
// pairing proof — and a single owning context owns the writer for the
// model's lifetime so `m_`/`d` outlive every call.
DistributedTireWriter::DistributedTireWriter(const mjModel *m,
                                             const mjData *d,
                                             nativecfg::TireConfig config)
    : m_(m), cfg_(std::move(config)), nv_(static_cast<int>(m->nv)) {
    if (m_->nv < 0 || m_->nv > std::numeric_limits<int>::max())
        throw std::invalid_argument(
            "distributed tire writer needs a model width in int range");
    // Dataclass __post_init__ validation happened when Python built the
    // config — before __init__ — so validate() precedes the ctor body.
    nativecfg::validate(cfg_);
    // distributed_tire_forces.py:37-38 — the backend gate.
    if (cfg_.backend != "distributed_2d_reference")
        throw std::invalid_argument(
            "distributed force applier requires its explicit backend");
    // :42-43 — the surface-map gate, keeping the oracle's wording.
    if (cfg_.surface_mode == "track") {
        if (!cfg_.surface_map)
            throw std::invalid_argument(
                "track material mode needs the actual track surface map");
        // SurfaceMap.__init__ (surface.py:160-206) sorts sections by
        // start_m; the section/spec/overlap checks are nativecfg::validate()'s
        // domain (config_validation.hpp:344-358), already run above.
        surface_map_ = *cfg_.surface_map;
        std::ranges::sort(surface_map_->sections, {},
                          [](const auto &sec) { return sec.start_m; });
    }
    // :40 — SignedProfile over the compiled terrain cross-section.
    profile_.emplace(biketyre::compiled_profile_vertices(m_, d, "terrain"));
    // :47-49 — geoms/bodies/radii for ('front','rear').
    geoms_ = {
        resolve_id(m_, mjOBJ_GEOM, "geom_front_contact"),
        resolve_id(m_, mjOBJ_GEOM, "geom_rear_contact")
    };
    const std::span<const int> body_of =
            model_access::readonly_buffer(m_->geom_bodyid, m_->ngeom);
    const std::span<const mjtNum> gsize = model_access::readonly_buffer(
        m_->geom_size, model_access::kXYZ * m_->ngeom);
    for (std::size_t i = 0; i < 2; ++i) {
        model_access::require_id(geoms_[i], m_->ngeom, "tire contact geom");
        bodies_[i] = body_of[static_cast<std::size_t>(geoms_[i])];
        radii_[i] =
                gsize[model_access::kXYZ * static_cast<std::size_t>(geoms_[i])];
    }
    // :52-55 — every geom on each wheel body must be collision-disabled for
    // the distributed backend.
    const std::span<const int> contype =
            model_access::readonly_buffer(m_->geom_contype, m_->ngeom);
    const std::span<const int> conaffinity =
            model_access::readonly_buffer(m_->geom_conaffinity, m_->ngeom);
    for (const int body: bodies_)
        for (int g = 0; g < m_->ngeom; ++g)
            if (body_of[static_cast<std::size_t>(g)] == body &&
                (contype[static_cast<std::size_t>(g)] ||
                 conaffinity[static_cast<std::size_t>(g)]))
                throw std::invalid_argument(
                    "native wheel collisions must be disabled for "
                    "the distributed backend");
    // :44-46 — the distributed section is engaged by the backend gate plus
    // validate(); station_angles owns the 16..512 domain.
    const nativecfg::DistributedConfig &dist = engaged(cfg_.distributed);
    stations_ = biketyre::station_angles(dist.station_count);
    const std::size_t n = stations_.angles.size();
    // :50-51 — local[side] = column_stack((r*cos(angles), 0, r*sin(angles))).
    // The numeric:: wrappers keep the same-operand pair as two separate
    // libm calls — the oracle's np.cos/np.sin can never fuse.
    for (std::size_t s = 0; s < 2; ++s) {
        local_[s].resize(n);
        for (std::size_t i = 0; i < n; ++i)
            local_[s][i] = {radii_[s] * numeric::np_cos(stations_.angles[i]), 0.0,
                            radii_[s] * numeric::np_sin(stations_.angles[i])};
    }
    // Spec §4.5 scratch — every container the Task-5 warm core fills, plus
    // the stored-energy and wire scratch this task already uses.
    const std::size_t nv = static_cast<std::size_t>(nv_);
    points_.resize(n * model_access::kXYZ);
    heights_.resize(n);
    active_ids_.resize(n);
    jac_center_.resize(model_access::kXYZ * nv);
    jr_.resize(model_access::kXYZ * nv);
    jac_station_.resize(model_access::kXYZ * nv);
    loads_.resize(n);
    energies_.resize(n);
    depths_.resize(n);
    normals_.resize(n * model_access::kXYZ);
    tmp_.resize(nv);
    qfrc_out_.resize(nv);
    states_[0].resize(n);
    states_[1].resize(n);
    states_stage_[0].resize(n);
    states_stage_[1].resize(n);
    states_pod_in_[0].resize(n);
    states_pod_in_[1].resize(n);
    states_pod_out_[0].resize(n);
    states_pod_out_[1].resize(n);
    auto model = std::make_shared<bike::physics_core::TyreModel<double>>();
    model->backend = bike::physics_core::TyreBackendKind::distributed_2d_reference;
    model->nq = static_cast<bike::physics_core::i32>(m_->nq);
    model->nv = nv_;
    model->nbody = static_cast<bike::physics_core::i32>(m_->nbody);
    model->njnt = static_cast<bike::physics_core::i32>(m_->njnt);
    model->station_count = static_cast<bike::physics_core::i32>(n);
    model->distinct_normal_cosine = numeric::np_cos(
        cfg_.distinct_normal_deg * (std::numbers::pi / 180.0));
    for (std::size_t s = 0; s < 2; ++s) {
        const nativecfg::TireParams &pcfg = s == 0 ? cfg_.front : cfg_.rear;
        const nativecfg::DistributedMaterial &domain =
            s == 0 ? dist.front : dist.rear;
        auto &w = std::span{model->wheels}[s];
        w.geom_id = geoms_[s];
        w.radius_m = radii_[s];
        w.tangent_k_n_m = pcfg.tangent_k_n_m;
        w.tire_mu = pcfg.mu;
        w.relaxation_length_m = pcfg.relaxation_length_m;
        w.station_weight = stations_.weights[0];
        w.valid_deflection_min_m = domain.valid_deflection_range_m[0];
        w.valid_deflection_max_m = domain.valid_deflection_range_m[1];
        const std::size_t knots = std::min(
            domain.density.knots_m.size(),
            domain.density.stiffness_n_m.size());
        if (knots > static_cast<std::size_t>(bike::physics_core::kMaxHingeKnots))
            throw std::invalid_argument(
                "distributed hinge density knot count exceeds kMaxHingeKnots");
        w.distributed_density.knot_count =
            static_cast<bike::physics_core::i32>(knots);
        w.distributed_density.damping_ns_m = domain.density.damping_ns_m;
        w.distributed_density.loss_factor = domain.density.loss_factor;
        w.distributed_density.hysteresis_rate_eps_mps =
            domain.density.hysteresis_rate_eps_mps;
        const std::span<double, bike::physics_core::kMaxHingeKnots> dst_k(
            w.distributed_density.knots_m);
        const std::span<double, bike::physics_core::kMaxHingeKnots> dst_s(
            w.distributed_density.stiffness_n_m);
        for (std::size_t k = 0; k < knots; ++k) {
            dst_k[k] = domain.density.knots_m[k];
            dst_s[k] = domain.density.stiffness_n_m[k];
        }
    }
    core_surface_map_complete_ = surface_map_
        ? tirecommon::fill_surface_map_view(&*surface_map_, model->surface_map)
        : true;
    core_model_ = std::move(model);
    const std::size_t cols = 2 * n * kLeaves.size();
    wire_cols_.resize(cols);
    wire_seen_.resize(cols);
    // :67-70 — the canonical wire columns, built once: side-major,
    // station-major, leaf-major, `f'{side}.s{index:03d}.{leaf}'`.
    state_names_.reserve(cols);
    canonical_.reserve(cols);
    for (const char *side: kSides)
        for (std::size_t i = 0; i < n; ++i)
            for (const char *leaf: kLeaves) {
                std::string name =
                        std::format("{}.s{:03d}.{}", side, i, leaf);
                canonical_.emplace(name, state_names_.size());
                state_names_.push_back(std::move(name));
            }
    // Task-5 commit capacity: one patch per active station, plus the literal
    // status/backend strings the marked commit assigns — the same
    // reserve-at-construction discipline TireWriter uses for its slots.
    constexpr std::string_view kBackend = "distributed_2d_reference";
    constexpr std::string_view kCapability =
            "experimental_reference_not_release_qualified";
    const std::size_t calibration_capacity = std::max(
            dist.front.calibration_status.size(),
            dist.rear.calibration_status.size());
    for (std::size_t s = 0; s < 2; ++s) {
        snapshot_stage_[s].patches.reserve(n);
        snapshots_[s].patches.reserve(n);
        probe_snapshots_[s].patches.reserve(n);
        snapshot_stage_[s].backend.reserve(kBackend.size());
        snapshots_[s].backend.reserve(kBackend.size());
        probe_snapshots_[s].backend.reserve(kBackend.size());
        diagnostics_stage_[s].calibration_status.reserve(
                calibration_capacity);
        diagnostics_[s].calibration_status.reserve(calibration_capacity);
        diagnostics_stage_[s].capability_status.reserve(kCapability.size());
        diagnostics_[s].capability_status.reserve(kCapability.size());
    }
    // reset() (:58-62) — the member initializers already hold these values.
}

DistributedTireWriter::~DistributedTireWriter() = default;
DistributedTireWriter::DistributedTireWriter(const DistributedTireWriter &) = default;
DistributedTireWriter &DistributedTireWriter::operator=(const DistributedTireWriter &) = default;
DistributedTireWriter::DistributedTireWriter(DistributedTireWriter &&) noexcept = default;
DistributedTireWriter &DistributedTireWriter::operator=(DistributedTireWriter &&) noexcept = default;

void DistributedTireWriter::reset() {
    // :58-62. Copy-assign from const empties (not move-assign or
    // re-construct): the empty source keeps every destination's reserved
    // patch/string capacity — the committed slots stay construction-sized
    // for the marked commit — and `committed_` carries the observable
    // "empty until the first advancing compute" state.
    for (auto &states: states_)
        std::ranges::fill(states, StationState{});
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

const std::vector<std::string> &DistributedTireWriter::state_names() const {
    return state_names_;
}

void DistributedTireWriter::set_state(std::span<const std::string> names,
                                      std::span<const double> row) {
    prepare_state(names, row);
    commit_prepared_state();
}

void DistributedTireWriter::prepare_state(std::span<const std::string> names,
                                         std::span<const double> row) {
    // :82-106 — the strict canonical-row contract documented in the header:
    // unknown columns first (in first-occurrence order, like the oracle),
    // then the equal-length/shape gate, then canonical coverage, then the
    // transactional decode. Every rejection is std::runtime_error — the
    // pinned RuntimeError surface of the native wire contract — with the
    // oracle's ValueError message texts preserved.
    std::ranges::fill(wire_seen_, static_cast<char>(0));
    for (const std::string &name: names) {
        const auto it = canonical_.find(name);
        if (it == canonical_.end())
            throw std::runtime_error("tire state has unknown column " + name);
        if (wire_seen_[it->second] != 0)
            throw std::runtime_error(
                    "tire state has duplicate column " + name);
        wire_seen_[it->second] = 1;
    }
    if (names.size() != row.size())
        throw std::runtime_error("tire state row has (" +
                                 std::to_string(row.size()) +
                                 ",) cells for " +
                                 std::to_string(names.size()) + " columns");
    for (std::size_t c = 0; c < state_names_.size(); ++c)
        if (wire_seen_[c] == 0)
            throw std::runtime_error("tire state is missing column " +
                                     state_names_[c]);
    // Scatter into canonical column order — the oracle gathers the same
    // values through its `index` map (:100).
    for (std::size_t i = 0; i < names.size(); ++i)
        wire_cols_[canonical_.find(names[i])->second] = row[i];
    // :93-104 — decode into the stage; the committed states move only after
    // every station validates.
    const std::size_t n = stations_.angles.size();
    for (std::size_t s = 0; s < 2; ++s) {
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t base = (s * n + i) * kLeaves.size();
            const double xi = wire_cols_[base];
            const double t0 = wire_cols_[base + 1];
            const double t1 = wire_cols_[base + 2];
            const double t2 = wire_cols_[base + 3];
            const bool all_nan = std::isnan(t0) && std::isnan(t1) &&
                                 std::isnan(t2);
            // Strictest reading (header CONTRACT): a partially nonfinite
            // tangent rejects unconditionally — even beside an inactive xi.
            if (!all_nan && (!std::isfinite(t0) || !std::isfinite(t1) ||
                             !std::isfinite(t2)))
                throw std::runtime_error(
                        "tire state has a partially nonfinite tangent at "
                        "column " +
                        state_names_[base + 1]);
            if (std::isnan(xi)) {
                // :101-102 — the station is simply absent from `states`.
                states_stage_[s][i] = StationState{};
                continue;
            }
            if (!std::isfinite(xi))
                throw std::runtime_error("tire state column " +
                                         state_names_[base] +
                                         " must be finite or NaN");
            StationState next;
            next.active = true;
            next.xi = xi;
            if (!all_nan)
                next.tangent = std::array<double, 3>{t0, t1, t2};
            states_stage_[s][i] = next;
        }
    }
}

void DistributedTireWriter::commit_prepared_state() noexcept {
    // Both banks have identical construction-sized storage; swapping also
    // makes this cross-owner initialization commit unconditionally no-throw.
    states_[0].swap(states_stage_[0]);
    states_[1].swap(states_stage_[1]);
    last_time_s_.reset();
}

std::vector<double> DistributedTireWriter::state() const {
    // :72-80 — one row of 2*N*4 doubles, NaN for every inactive station and
    // every unset tangent.
    const std::size_t n = stations_.angles.size();
    std::vector<double> out(2 * n * kLeaves.size(),
                            std::numeric_limits<double>::quiet_NaN());
    for (std::size_t s = 0; s < 2; ++s)
        for (std::size_t i = 0; i < n; ++i) {
            const StationState &state = states_[s][i];
            if (!state.active)
                continue;
            const std::size_t base = (s * n + i) * kLeaves.size();
            out[base] = state.xi;
            if (state.tangent.has_value())
                for (std::size_t k = 0; k < model_access::kXYZ; ++k)
                    out[base + 1 + k] = (*state.tangent)[k];
        }
    return out;
}

// _stations (distributed_tire_forces.py:108-115) for one wheel: the world
// lift, the batch height evaluation, and the penetrating ids — shared by
// stored_energy below and the Task-5 tick. `points_`/`heights_`/`active_ids_`
// are mutable scratch so the const stored_energy can reuse the same
// construction-sized buffers.
std::size_t DistributedTireWriter::station_points(const mjData *d,
                                                  std::size_t side) const {
    const std::size_t n = stations_.angles.size();
    const std::size_t g3 =
            model_access::kXYZ * static_cast<std::size_t>(geoms_[side]);
    const std::size_t b9 = 9 * static_cast<std::size_t>(bodies_[side]);
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
        d->geom_xpos, model_access::kXYZ * m_->ngeom);
    const std::span<const mjtNum> xmat = model_access::readonly_buffer(
        d->xmat, 9 * m_->nbody);
    // center = data.geom_xpos[geom] (:110); rotation = xmat[body] (:111) is
    // the row-major 3x3 the (N,3) product transposes.
    const std::array<double, model_access::kXYZ> center = {xpos[g3], xpos[g3 + 1],
                                             xpos[g3 + 2]};
    blas::dgemm(blas::Order::row_major, blas::Transpose::no,
                blas::Transpose::yes, static_cast<int>(n),
                static_cast<int>(model_access::kXYZ), static_cast<int>(model_access::kXYZ), 1.0,
                &local_[side][0][0], static_cast<int>(model_access::kXYZ),
                xmat.subspan(b9, 9).data(), static_cast<int>(model_access::kXYZ), 0.0,
                points_.data(), static_cast<int>(model_access::kXYZ));
    // :112 — `center + local @ rotation.T`; addition commutes bitwise, so
    // += writes the same doubles the oracle's broadcast add produces.
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t k = 0; k < model_access::kXYZ; ++k)
            points_[model_access::kXYZ * i + k] += center[k];
    // :113 — profile.height over the WHOLE X column first: a domain
    // rejection aborts before any station is judged active.
    for (std::size_t i = 0; i < n; ++i)
        heights_[i] = engaged(profile_).height(points_[model_access::kXYZ * i]);
    // :114 — np.flatnonzero: the penetrating ids, ascending.
    std::size_t count = 0;
    for (std::size_t i = 0; i < n; ++i)
        if (points_[model_access::kXYZ * i + 2] < heights_[i])
            active_ids_[count++] = static_cast<int>(i);
    return count;
}

// stored_energy (:117-129): radial density energy over the CURRENT
// geometry's penetrating stations, then the stored shear energy of every
// active station — sorted(self.states.items()) is exactly the side-major,
// station-ascending order the `active` flag enumerates here.
double DistributedTireWriter::stored_energy(const mjData *d) const {
    if (d == nullptr)
        throw std::invalid_argument(
            "tire stored_energy needs a live mjData");
    const nativecfg::DistributedConfig &dist = engaged(cfg_.distributed);
    double energy = 0.0;
    for (std::size_t s = 0; s < 2; ++s) {
        const nativecfg::DistributedMaterial &domain =
                s == 0 ? dist.front : dist.rear;
        const std::size_t count = station_points(d, s);
        for (std::size_t j = 0; j < count; ++j) {
            const std::size_t sid =
                    static_cast<std::size_t>(active_ids_[j]);
            // :123-124 — query the (x, z) pair; `penetration` is -distance.
            const biketyre::PointDistance geometry =
                    engaged(profile_).query(
                        {points_[model_access::kXYZ * sid], points_[model_access::kXYZ * sid + 2]});
            energy += stations_.weights[sid] *
                      biketyre::station_density(-geometry.distance,
                                                domain.density)
                              .second;
        }
    }
    // :127-128 — `.5 * tangent_k * weight * xi**2` keeps the oracle's
    // left-associative multiply order; `xi ** 2` lowers to libm pow (the
    // pyfloat::pow boundary) — never the x*x fold.
    for (std::size_t s = 0; s < 2; ++s) {
        const double tk = s == 0 ? cfg_.front.tangent_k_n_m
                                 : cfg_.rear.tangent_k_n_m;
        for (std::size_t i = 0; i < stations_.angles.size(); ++i) {
            const StationState &state = states_[s][i];
            if (!state.active)
                continue;
            energy += .5 * tk * stations_.weights[i] *
                      pyfloat::pow(state.xi, 2.0);
        }
    }
    return energy;
}

// Caller precondition: `d` is the live mjData of the model this writer was
// built on — the single owning context's contract, same as TireWriter.
void DistributedTireWriter::compute_into(const mjData *d, double dt,
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
// allocates — every buffer the kernel touches is construction-sized.
CoreStatus DistributedTireWriter::try_compute_into(
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

std::vector<double> DistributedTireWriter::qfrc(const mjData *d, double dt) {
    compute_into(d, dt, qfrc_out_);
    return qfrc_out_;
}

// compute_qfrc(advance=True) (distributed_tire_forces.py:139-218). The
// interval scalar gate and the once-per-timestamp gate live here, like
// TireWriter's, so compute_into and try_compute_into share them. Every
// per-side artifact is staged first and committed only after BOTH wheels
// evaluate — a throwing side leaves states, snapshots, diagnostics,
// energies and the clock untouched, matching the oracle's local-dict
// build.
//
// Lowering decisions (same-libcall contract, verified bitwise by
// tests/reference/test_native_distributed_tire.py):
//   jp_center @ qvel / jac @ qvel   → cblas_dgemv(RowMajor, NoTrans)
//   jac.T @ force                   → cblas_dgemv(RowMajor, Trans) into
//                                     tmp_, then elementwise += (the
//                                     oracle materializes the product
//                                     before the += — never fused)
//   velocity/tangent/center dots    → blas::ddot on (3,) rows
//   np.sum(loads) / np.sum(energies)→ numeric::numpy_sum (the np.sum
//                                     reduce identity over pairwise
//                                     order)
//   np.cos(np.radians(x))           → numeric::np_cos(x * (pi/180)):
//                                     np.radians is one multiply and
//                                     the ufunc contract is npy_cos
//                                     (the numeric_sincos rationale)
//   xi ** 2 / old.xi ** 2           → pyfloat::pow — the libm call, not
//                                     the x*x compiler fold
//   scalar ** nothing else          → literal multiplies elsewhere
//   max(0., x)                      → std::max (identical a<b?b:a
//                                     semantics, NaN included)
void DistributedTireWriter::accumulate_validated(const mjData *d,
                                                 double dt_arg,
                                                 std::span<double> out) {
    // scalar(dt, 'distributed tire interval', positive=True) (:139) and
    // the double-advance gate (:141-142).
    if (!std::isfinite(dt_arg) || dt_arg <= 0.0)
        throw std::invalid_argument("invalid distributed tire interval");
    const double dt = dt_arg;
    const double time = static_cast<double>(d->time);
    if (last_time_s_.has_value() && time <= *last_time_s_)
        throw std::invalid_argument(
            "tire state can advance only once per increasing timestamp");
    // :143-146 — zeros(nv) result, the tick accumulators, and the
    // distinct-normal cosine threshold.
    std::ranges::fill(out, 0.0);
    const nativecfg::DistributedConfig &dist = engaged(cfg_.distributed);
    const std::size_t n = stations_.angles.size();
    constexpr std::size_t kDim =
            static_cast<std::size_t>(model_access::kXYZ);
    double energy = 0.0, total_loss = 0.0, radial_loss_power = 0.0;
    const double cos_distinct = numeric::np_cos(
        cfg_.distinct_normal_deg * (std::numbers::pi / 180.0));
    const std::span<const mjtNum> xpos = model_access::readonly_buffer(
        d->geom_xpos, model_access::kXYZ * m_->ngeom);
    const std::span<const mjtNum> xmat = model_access::readonly_buffer(
        d->xmat, 9 * m_->nbody);
    const std::span<const mjtNum> qvel =
            model_access::readonly_buffer(d->qvel, m_->nv);

    struct CpuDistributedWorkspace {
        double *points;
        bike::physics_core::i32 *active_ids;
        double *jac_center;
        double *jr;
        double *jac_station;
        double *tmp;
    };

    struct DistributedCpuIo {
        DistributedTireWriter *self = nullptr;
        const nativecfg::DistributedConfig *dist_cfg = nullptr;
        double time_val = 0.0;
        double dt_val = 0.0;
        std::size_t n_stations = 0;
        double cos_distinct_val = 0.0;
        double *energy_acc = nullptr;
        double *total_loss_acc = nullptr;
        double *radial_power_acc = nullptr;
        int nonsmooth_count = 0;
        std::exception_ptr *caught = nullptr;

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
                const auto [mu_val, material_name] =
                    tirecommon::effective_friction(
                        query.tire_mu,
                        self->surface_map_ ? &*self->surface_map_ : nullptr,
                        query.contact_x, query.slip);
                static_cast<void>(material_name);
                out_val.mu = mu_val;
                out_val.section_idx = -1;
                out_status = bike::physics_core::Status::ok;
                err = bike::physics_core::ErrCode::none;
            } catch (...) {
                *caught = std::current_exception();
                out_status = bike::physics_core::Status::invalid_argument;
                err = bike::physics_core::ErrCode::outside_domain;
            }
            return true;
        }

        bike::physics_core::Status record_station(
            bike::physics_core::i32 side,
            const bike::physics_core::DistributedStationRecord<double> &rec,
            bike::physics_core::ErrCode &err
        ) noexcept {
            try {
                const std::size_t s_idx = static_cast<std::size_t>(side);
                const std::size_t j_idx = static_cast<std::size_t>(rec.j);
                const Vec3 point = {rec.point.x, rec.point.y, rec.point.z};
                const Vec3 normal = {rec.normal.x, rec.normal.y, rec.normal.z};
                self->loads_[j_idx] = rec.load;
                self->energies_[j_idx] = rec.elastic_energy;
                self->depths_[j_idx] = rec.penetration;
                nonsmooth_count += rec.geometry.nonsmooth != 0 ? 1 : 0;
                self->normals_[kDim * j_idx] = normal[0];
                self->normals_[kDim * j_idx + 1] = normal[1];
                self->normals_[kDim * j_idx + 2] = normal[2];
                TireSnapshot &snap = self->snapshot_stage_[s_idx];
                if (snap.patches.size() >= n_stations)
                    throw std::invalid_argument(
                        "tire snapshot patch capacity exceeded");
                snap.patches.push_back(
                    make_patch(point, normal, rec.load, rec.tangent_force,
                               rec.slip));
                err = bike::physics_core::ErrCode::none;
                return bike::physics_core::Status::ok;
            } catch (...) {
                *caught = std::current_exception();
                err = bike::physics_core::ErrCode::overflow_nonfinite;
                return bike::physics_core::Status::invalid_argument;
            }
        }

        bike::physics_core::Status record_wheel(
            bike::physics_core::i32 side,
            const bike::physics_core::DistributedWheelRecord<double> &rec,
            bike::physics_core::ErrCode &err
        ) noexcept {
            try {
                const std::size_t s_idx = static_cast<std::size_t>(side);
                const std::size_t count =
                    static_cast<std::size_t>(rec.active_count);
                const nativecfg::DistributedMaterial &domain =
                    s_idx == 0 ? dist_cfg->front : dist_cfg->rear;
                const Vec3 center = {
                    rec.center.x, rec.center.y, rec.center.z};

                const double normal_total = numeric::numpy_sum(
                    std::span{self->loads_}.first(count));
                const double radial_energy = numeric::numpy_sum(
                    std::span{self->energies_}.first(count));
                const double max_depth =
                    count > 0 ? *std::ranges::max_element(
                                    self->depths_.begin(),
                                    self->depths_.begin() +
                                        static_cast<std::ptrdiff_t>(count))
                              : 0.0;
                const double center_clearance =
                    engaged(self->profile_)
                            .query({center[0], center[2]})
                            .distance -
                    self->radii_[s_idx];
                const double signed_overlap =
                    count > 0 ? max_depth
                              : -std::max(0.0, center_clearance);

                bool distinct = false;
                if (count > 1)
                    for (std::size_t a = 0; a < count && !distinct; ++a)
                        for (std::size_t b = 0; b < count; ++b)
                            if (dot3({self->normals_[kDim * a],
                                      self->normals_[kDim * a + 1],
                                      self->normals_[kDim * a + 2]},
                                     {self->normals_[kDim * b],
                                      self->normals_[kDim * b + 1],
                                      self->normals_[kDim * b + 2]}) <
                                cos_distinct_val) {
                                distinct = true;
                                break;
                            }

                bool separate = false;
                if (count > 1) {
                    for (std::size_t i = 0; i + 1 < count; ++i)
                        if (self->active_ids_[i + 1] -
                                self->active_ids_[i] > 1) {
                            separate = true;
                            break;
                        }
                    if (self->active_ids_[0] == 0 &&
                        self->active_ids_[count - 1] ==
                            static_cast<int>(n_stations) - 1)
                        separate = false;
                }

                TireSnapshot &snap = self->snapshot_stage_[s_idx];
                snap.time_s = time_val;
                snap.backend = "distributed_2d_reference";
                snap.geometric_contact = count > 0;
                snap.wheel_axis_m = center;
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
                snap.effective_radius_m =
                    effective_radius(snap.patches, center);

                TireDiagnostics &diag = self->diagnostics_stage_[s_idx];
                diag.multi_support = distinct || separate;
                diag.penetration_m = signed_overlap;
                diag.normal_load_n = normal_total;
                diag.supports_multiple_contacts = true;
                diag.capability_status =
                    "experimental_reference_not_release_qualified";
                diag.calibration_status = domain.calibration_status;
                diag.station_count = static_cast<int>(n_stations);
                diag.active_station_count = static_cast<int>(count);
                diag.nonsmooth_station_count = nonsmooth_count;
                diag.max_patch_count = static_cast<int>(count);
                diag.outside_material_load_range =
                    !(domain.valid_load_range_n[0] <= normal_total &&
                      normal_total <= domain.valid_load_range_n[1]);
                diag.outside_material_deflection_range =
                    !(domain.valid_deflection_range_m[0] <= max_depth &&
                      max_depth <= domain.valid_deflection_range_m[1]);
                diag.radial_energy_j = radial_energy;
                diag.shear_energy_j = rec.shear_energy;
                diag.brush_loss_j = rec.losses;

                *energy_acc += radial_energy + rec.shear_energy;
                *total_loss_acc += rec.losses;
                *radial_power_acc += rec.radial_power;
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
    auto profile_height_fn = [&](double x, double &out_h,
                                 bike::physics_core::ErrCode &err) noexcept
        -> bike::physics_core::Status {
        try {
            out_h = engaged(profile_).height(x);
            err = bike::physics_core::ErrCode::none;
            return bike::physics_core::Status::ok;
        } catch (...) {
            caught_err = std::current_exception();
            err = bike::physics_core::ErrCode::outside_domain;
            return bike::physics_core::Status::invalid_argument;
        }
    };

    auto signed_query_fn = [&](const bike::physics_core::TyreXZ<double> &xz,
                               bike::physics_core::PointDistanceResult<double> &out_geom,
                               bike::physics_core::ErrCode &err) noexcept
        -> bike::physics_core::Status {
        try {
            const biketyre::PointDistance g =
                engaged(profile_).query({xz.x, xz.z});
            out_geom.distance = g.distance;
            out_geom.normal[0] = g.normal[0];
            out_geom.normal[1] = g.normal[1];
            out_geom.point[0] = g.point[0];
            out_geom.point[1] = g.point[1];
            out_geom.segment = g.segment;
            out_geom.nonsmooth = g.nonsmooth ? 1 : 0;
            err = bike::physics_core::ErrCode::none;
            return bike::physics_core::Status::ok;
        } catch (...) {
            caught_err = std::current_exception();
            err = bike::physics_core::ErrCode::outside_domain;
            return bike::physics_core::Status::invalid_argument;
        }
    };

    std::size_t current_side = 0;
    auto center_jac_fn = [&](const bike::physics_core::TyreVec3<double> &c_v,
                             double * /*jp_ptr*/,
                             double * /*jr_ptr*/,
                             bike::physics_core::ErrCode &err) noexcept
        -> bike::physics_core::Status {
        try {
            const Vec3 c_vec = {c_v.x, c_v.y, c_v.z};
            point_jacobian(m_, d, bodies_[current_side], c_vec,
                           jac_center_, jr_);
            err = bike::physics_core::ErrCode::none;
            return bike::physics_core::Status::ok;
        } catch (...) {
            caught_err = std::current_exception();
            err = bike::physics_core::ErrCode::out_of_bounds_chain;
            return bike::physics_core::Status::invalid_argument;
        }
    };

    CpuDistributedWorkspace ws{
        .points = points_.data(),
        .active_ids = active_ids_.data(),
        .jac_center = jac_center_.data(),
        .jr = jr_.data(),
        .jac_station = jac_station_.data(),
        .tmp = tmp_.data()
    };

    DistributedCpuIo cpu_io{
        .self = this,
        .dist_cfg = &dist,
        .time_val = time,
        .dt_val = dt,
        .n_stations = n,
        .cos_distinct_val = cos_distinct,
        .energy_acc = &energy,
        .total_loss_acc = &total_loss,
        .radial_power_acc = &radial_loss_power,
        .nonsmooth_count = 0,
        .caught = &caught_err
    };

    // :147 — self.bodies.items() — front then rear, insertion order.
    for (std::size_t s = 0; s < 2; ++s) {
        current_side = s;
        cpu_io.nonsmooth_count = 0;
        snapshot_stage_[s].patches.clear();
        const std::size_t g3 =
                kDim * static_cast<std::size_t>(geoms_[s]);
        const Vec3 center = {xpos[g3], xpos[g3 + 1], xpos[g3 + 2]};
        const std::size_t b9 = 9 * static_cast<std::size_t>(bodies_[s]);

        for (std::size_t i = 0; i < n; ++i) {
            const StationState &st_in = states_[s][i];
            auto &pod = states_pod_in_[s][i];
            pod.active = st_in.active ? 1 : 0;
            pod.xi = st_in.xi;
            if (st_in.tangent.has_value()) {
                std::ranges::copy(*st_in.tangent, std::span{pod.tangent}.begin());
                pod.has_tangent = 1;
            } else {
                pod.tangent[0] = 0.0;
                pod.tangent[1] = 0.0;
                pod.tangent[2] = 0.0;
                pod.has_tangent = 0;
            }
        }

        auto err = bike::physics_core::ErrCode::none;
        const auto st = bike::physics_core::step_distributed_wheel<
            double, bike::physics_core::CpuExactToolkit>(
                core_model_.get(),
                static_cast<bike::physics_core::i32>(s),
                dt,
                center.data(),
                xmat.subspan(b9, 9).data(),
                qvel.data(),
                &local_[s][0][0],
                stations_.weights.data(),
                states_pod_in_[s].data(),
                states_pod_out_[s].data(),
                out.data(),
                ws,
                profile_height_fn,
                signed_query_fn,
                center_jac_fn,
                cpu_io,
                err);
        if (st != bike::physics_core::Status::ok) {
            if (caught_err) {
                std::rethrow_exception(caught_err);
            }
            if (err == bike::physics_core::ErrCode::overflow_passivity) {
                throw std::overflow_error(
                    "brush update violates discrete passivity");
            }
            if (st == bike::physics_core::Status::overflow) {
                throw std::overflow_error(
                    "tire calculation exceeds finite floating-point range");
            }
            throw std::invalid_argument("distributed wheel invalid argument");
        }

        for (std::size_t i = 0; i < n; ++i) {
            const auto &pod = states_pod_out_[s][i];
            if (pod.active != 0) {
                states_stage_[s][i] = StationState{
                    .active = true,
                    .xi = pod.xi,
                    .tangent = Vec3{pod.tangent[0], pod.tangent[1], pod.tangent[2]}
                };
            } else {
                states_stage_[s][i] = StationState{};
            }
        }
    }
    // :215-217 — the atomic commit after both wheels evaluate. The
    // committed slots copy-assign from the stages — capacity-reserved
    // at construction, so the marked path never allocates — and
    // committed_ flips the observable published state.
    states_[0] = states_stage_[0];
    states_[1] = states_stage_[1];
    snapshots_[0] = snapshot_stage_[0];
    snapshots_[1] = snapshot_stage_[1];
    diagnostics_[0] = diagnostics_stage_[0];
    diagnostics_[1] = diagnostics_stage_[1];
    committed_ = true;
    elastic_energy_j_ = energy;
    brush_loss_step_j_ = total_loss;
    radial_dissipation_power_w_ = radial_loss_power;
    last_time_s_ = time;
}

// compute_qfrc(advance=False) (:131-138): a detached copy evaluates the
// advancing kernel on a cleared clock; only its committed snapshots cross
// back. Cold path — the copy duplicates the construction-sized scratch.
void DistributedTireWriter::probe_compute_into(const mjData *d, double dt,
                                               std::span<double> out) {
    if (d == nullptr)
        throw std::invalid_argument("tire probe needs a live mjData");
    if (out.size() != static_cast<std::size_t>(nv_) ||
        (out.data() == nullptr && !out.empty()))
        throw std::invalid_argument("tire output width");
    DistributedTireWriter probe(*this);
    probe.last_time_s_.reset();
    probe.accumulate_validated(d, dt, out);
    probe_snapshots_ = probe.snapshots_;
    probe_committed_ = probe.committed_;
}

std::vector<double> DistributedTireWriter::probe_qfrc(const mjData *d,
                                                    double dt) {
    std::vector<double> out(static_cast<std::size_t>(nv_));
    probe_compute_into(d, dt, out);
    return out;
}
