// writers/distributed_tire.cpp — port of distributed_tire_forces.py:
// construction, the canonical per-station state wire, stored energy, and the
// reset/clock/snapshot/diagnostic scaffolding (plan Task 4; the tick itself
// is Task 5). Comments cite file:line at porting time.
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
#include "../model_access.hpp"
#include "../numeric_sincos.hpp"
#include "../tyre/profile.hpp"
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
#include <string_view>
#include <utility>
#include <vector>

namespace {
    // physical_mapping.py:6-11 — resolve_id. Same plain name check
    // TireWriter applies to the wheel contact geoms.
    int resolve_id(const mjModel *m, mjtObj kind, const char *name) {
        return model_access::resolve_id(m, kind, name);
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
            local_[s][i] = {radii_[s] * numeric::cos(stations_.angles[i]), 0.0,
                            radii_[s] * numeric::sin(stations_.angles[i])};
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
    // :105-106 — commit: copy-assign keeps the committed vectors' capacity.
    states_[0] = states_stage_[0];
    states_[1] = states_stage_[1];
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

// The warm-core twin of compute_into, in its final Task-5 shape:
// precondition gates report CoreStatus::invalid_input and residual kernel
// rejections are caught and mapped so noexcept is proved by the catch-all.
// Task 4 boundary: the kernel is still the stub, so a valid call surfaces
// the stub's logic_error through the std::exception arm as invalid_input —
// documented interim behavior until the station loop lands.
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

// Kernel scaffold — :139-142's interval scalar gate and the once-per-
// timestamp gate live here, like TireWriter's, so compute_into and
// try_compute_into share them. The station loop (spec §4.3) is Task 5.
// The stub unconditionally throws, which reads as noreturn to the
// -Wmissing-noreturn sweep — suppress it just for the stub body; Task 5's
// real kernel returns normally and drops the suppression.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wmissing-noreturn")
void DistributedTireWriter::accumulate_validated(const mjData *d,
                                                 double dt_arg,
                                                 std::span<double> out) {
    // scalar(dt, 'distributed tire interval', positive=True) (:139).
    if (!std::isfinite(dt_arg) || dt_arg <= 0.0)
        throw std::invalid_argument("invalid distributed tire interval");
    const double time = static_cast<double>(d->time);
    if (last_time_s_.has_value() && time <= *last_time_s_)
        throw std::invalid_argument(
            "tire state can advance only once per increasing timestamp");
    static_cast<void>(time);
    static_cast<void>(out);
    throw std::logic_error(
        "distributed tire tick is not implemented until Task 5");
}
NATIVE_DIAG_POP

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
