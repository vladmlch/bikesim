// native/src/physics_core/tyre_distributed.hpp — Tri-target Slice 2
// (distributed_2d_reference) station laws and wheel step kernel parameterized
// over Workspace, ProfileHeightFn, SignedQueryFn, CenterJacFn, and Io.
#pragma once

#include "common.hpp"
#include "tyre_contact.hpp"
#include "tyre_types.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL kernel: templated pointer parameters accept device/constant/thread buffers in MSL without address-space cast errors. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

template <class Real, class TK>
BIKE_HD Status station_density(
    Real delta,
    const HingeDensityView<Real> &density,
    BIKE_THREAD Real &out_force,
    BIKE_THREAD Real &out_energy,
    BIKE_THREAD ErrCode &err
) noexcept {
    if (!TK::isfinite(delta)) {
        err = ErrCode::outside_domain;
        return Status::invalid_argument;
    }
    Real force = BIKE_REAL(Real, 0.0);
    Real energy = BIKE_REAL(Real, 0.0);
    const i32 knots = density.knot_count < kMaxHingeKnots
        ? density.knot_count
        : kMaxHingeKnots;
    for (i32 i = 0; i < knots; ++i) {
        const Real x = TK::max(delta - density.knots_m[i], BIKE_REAL(Real, 0.0));
        force += density.stiffness_n_m[i] * x;
        energy += density.stiffness_n_m[i] * x * x;
    }
    out_force = force;
    out_energy = BIKE_REAL(Real, 0.5) * energy;
    err = ErrCode::none;
    return Status::ok;
}

template <class Real, class TK>
BIKE_HD Status station_response(
    Real delta,
    Real rate,
    Real weight,
    const HingeDensityView<Real> &density,
    BIKE_THREAD Real &out_force,
    BIKE_THREAD Real &out_energy,
    BIKE_THREAD ErrCode &err
) noexcept {
    if (!(TK::isfinite(delta) && TK::isfinite(rate) && TK::isfinite(weight)) ||
        weight < BIKE_REAL(Real, 0.0)) {
        err = ErrCode::invalid_station_quad;
        return Status::invalid_argument;
    }
    Real elastic = BIKE_REAL(Real, 0.0);
    Real energy = BIKE_REAL(Real, 0.0);
    const Status st =
        station_density<Real, TK>(delta, density, elastic, energy, err);
    if (st != Status::ok) {
        return st;
    }
    const Real force =
        delta > BIKE_REAL(Real, 0.0)
            ? TK::max(
                  BIKE_REAL(Real, 0.0),
                  elastic +
                      density.loss_factor * elastic *
                          TK::tanh(
                              TK::div(rate, density.hysteresis_rate_eps_mps)))
            : BIKE_REAL(Real, 0.0);
    out_force = weight * force;
    out_energy = weight * energy;
    err = ErrCode::none;
    return Status::ok;
}

template <
    class Real,
    u32 MaxStations = static_cast<u32>(kMaxGpuTyreStations),
    u32 MaxNv = static_cast<u32>(kMaxNv)>
struct GpuDistributedWorkspace {
    Real points[3u * MaxStations]{};
    i32 active_ids[MaxStations]{};
    Real jac_center[3u * MaxNv]{};
    Real jr[3u * MaxNv]{};
    Real jac_station[3u * MaxNv]{};
    Real tmp[MaxNv]{};
};

template <class Real>
struct DistributedStationRecord {
    i32 j = 0;
    i32 sid = 0;
    TyreVec3<Real> point{};
    TyreVec3<Real> normal{};
    PointDistanceResult<Real> geometry{};
    Real load = BIKE_REAL(Real, 0.0);
    Real elastic_energy = BIKE_REAL(Real, 0.0);
    Real penetration = BIKE_REAL(Real, 0.0);
    Real tangent_force = BIKE_REAL(Real, 0.0);
    Real slip = BIKE_REAL(Real, 0.0);
};

template <class Real>
struct DistributedWheelRecord {
    TyreVec3<Real> center{};
    i32 active_count = 0;
    Real losses = BIKE_REAL(Real, 0.0);
    Real shear_energy = BIKE_REAL(Real, 0.0);
    Real radial_power = BIKE_REAL(Real, 0.0);
};

template <class Real, class Io>
struct DistributedIoTraits {
    BIKE_HD static bool custom_friction(
        BIKE_THREAD Io &io,
        const FrictionQuery<Real> &query,
        BIKE_THREAD FrictionResult<Real> &out,
        BIKE_THREAD Status &out_status,
        BIKE_THREAD ErrCode &err
    ) noexcept {
        return io.custom_friction(query, out, out_status, err);
    }

    BIKE_HD static Status record_station(
        BIKE_THREAD Io &io,
        i32 side,
        BIKE_THREAD const DistributedStationRecord<Real> &rec,
        BIKE_THREAD ErrCode &err
    ) noexcept {
        return io.record_station(side, rec, err);
    }

    BIKE_HD static Status record_wheel(
        BIKE_THREAD Io &io,
        i32 side,
        BIKE_THREAD const DistributedWheelRecord<Real> &rec,
        BIKE_THREAD ErrCode &err
    ) noexcept {
        return io.record_wheel(side, rec, err);
    }
};

template <class Real>
struct DistributedIoTraits<Real, NoOpIo> {
    BIKE_HD static bool custom_friction(
        BIKE_THREAD NoOpIo &,
        const FrictionQuery<Real> &,
        BIKE_THREAD FrictionResult<Real> &,
        BIKE_THREAD Status &,
        BIKE_THREAD ErrCode &
    ) noexcept {
        return false;
    }

    BIKE_HD static Status record_station(
        BIKE_THREAD NoOpIo &, i32,
        BIKE_THREAD const DistributedStationRecord<Real> &,
        BIKE_THREAD ErrCode &
    ) noexcept {
        return Status::ok;
    }

    BIKE_HD static Status record_wheel(
        BIKE_THREAD NoOpIo &, i32,
        BIKE_THREAD const DistributedWheelRecord<Real> &,
        BIKE_THREAD ErrCode &
    ) noexcept {
        return Status::ok;
    }
};

template <
    class Real,
    class TK,
    class ModelPtr,
    class CenterPtr,
    class XmatPtr,
    class QvelPtr,
    class LocalXyzPtr,
    class WeightsPtr,
    class StateInPtr,
    class StateOutPtr,
    class QfrcPtr,
    class Workspace,
    class ProfileHeightFn,
    class SignedQueryFn,
    class CenterJacFn,
    class Io = NoOpIo>
BIKE_HD Status step_distributed_wheel(
    ModelPtr model,
    i32 side,
    Real dt,
    CenterPtr center_in,
    XmatPtr wheel_xmat,
    QvelPtr qvel,
    LocalXyzPtr local_xyz,
    WeightsPtr weights,
    StateInPtr prev_states,
    StateOutPtr next_states,
    QfrcPtr out_qfrc,
    BIKE_THREAD Workspace &ws,
    BIKE_THREAD ProfileHeightFn &profile_height_fn,
    BIKE_THREAD SignedQueryFn &signed_query_fn,
    BIKE_THREAD CenterJacFn &center_jac_fn,
    BIKE_THREAD Io &io,
    ErrCode &err
) noexcept {
    const i32 nv = model->nv;
    const i32 station_count = model->station_count;
    const Real tk = model->wheels[side].tangent_k_n_m;
    const Real tire_mu = model->wheels[side].tire_mu;
    const Real relax_len = model->wheels[side].relaxation_length_m;
    const Real uniform_weight = model->wheels[side].station_weight;
    const HingeDensityView<Real> density =
        model->wheels[side].distributed_density;

    const Real center[3] = {center_in[0], center_in[1], center_in[2]};
    TK::ring_lift(station_count, local_xyz, wheel_xmat, ws.points);
    for (i32 i = 0; i < station_count; ++i) {
        for (i32 k = 0; k < 3; ++k) {
            ws.points[3 * i + k] += center[k];
        }
    }

    ErrCode local_err = ErrCode::none;
    i32 count = 0;
    for (i32 i = 0; i < station_count; ++i) {
        Real h = BIKE_REAL(Real, 0.0);
        const Status hst = profile_height_fn(ws.points[3 * i], h, local_err);
        if (hst != Status::ok) {
            err = local_err;
            return hst;
        }
        if (ws.points[3 * i + 2] < h) {
            ws.active_ids[count++] = i;
        }
    }

    const TyreVec3<Real> center_vec{center[0], center[1], center[2]};
    Status st = center_jac_fn(center_vec, ws.jac_center, ws.jr, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }
    Real center_velocity[3]{};
    TK::gemv_n3(nv, ws.jac_center, qvel, center_velocity);

    Real losses = BIKE_REAL(Real, 0.0);
    Real shear_energy = BIKE_REAL(Real, 0.0);
    Real radial_power = BIKE_REAL(Real, 0.0);

    for (i32 j = 0; j < count; ++j) {
        const i32 sid = ws.active_ids[j];
        const Real point[3] = {
            ws.points[3 * sid],
            ws.points[3 * sid + 1],
            ws.points[3 * sid + 2]
        };
        PointDistanceResult<Real> geometry{};
        const TyreXZ<Real> query_xz{point[0], point[2]};
        st = signed_query_fn(query_xz, geometry, local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }
        const Real normal[3] = {
            geometry.normal[0],
            BIKE_REAL(Real, 0.0),
            geometry.normal[1]
        };
        const Real tangent[3] = {normal[2], BIKE_REAL(Real, 0.0), -normal[0]};
        const Real arm[3] = {
            point[0] - center[0],
            point[1] - center[1],
            point[2] - center[2]
        };
        for (i32 k = 0; k < 3; ++k) {
            const i32 a = (k + 1) % 3;
            const i32 b = (k + 2) % 3;
            for (i32 i = 0; i < nv; ++i) {
                ws.jac_station[k * nv + i] =
                    ws.jac_center[k * nv + i] +
                    (ws.jr[a * nv + i] * arm[b] - ws.jr[b * nv + i] * arm[a]);
            }
        }
        Real velocity[3]{};
        TK::gemv_n3(nv, ws.jac_station, qvel, velocity);
        const Real rate =
            -((velocity[0] * normal[0] + velocity[1] * normal[1]) +
              velocity[2] * normal[2]);
        const Real penetration = -geometry.distance;
        const Real w_sid =
            weights != nullptr ? weights[sid] : uniform_weight;

        Real load = BIKE_REAL(Real, 0.0);
        Real elastic_energy = BIKE_REAL(Real, 0.0);
        st = station_response<Real, TK>(
            penetration, rate, w_sid, density, load, elastic_energy, local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }
        Real elastic_density = BIKE_REAL(Real, 0.0);
        Real unused_density_energy = BIKE_REAL(Real, 0.0);
        st = station_density<Real, TK>(
            penetration, density, elastic_density, unused_density_energy,
            local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }
        radial_power += TK::max(
            BIKE_REAL(Real, 0.0), (load - w_sid * elastic_density) * rate);

        Real xi = prev_states[sid].active != 0
            ? prev_states[sid].xi
            : BIKE_REAL(Real, 0.0);
        const Real k_station = tk * w_sid;
        if (prev_states[sid].active != 0 &&
            prev_states[sid].has_tangent != 0) {
            const Real old_tangent[3] = {
                prev_states[sid].tangent[0],
                prev_states[sid].tangent[1],
                prev_states[sid].tangent[2]
            };
            Real transported = BIKE_REAL(Real, 0.0);
            transport_bristle<Real, TK>(old_tangent, tangent, xi, transported);
            losses += TK::max(
                BIKE_REAL(Real, 0.0),
                BIKE_REAL(Real, 0.5) * k_station *
                    (xi * xi - transported * transported));
            xi = transported;
        }

        const Real slip = TK::dot3(velocity, tangent);
        Real mu = BIKE_REAL(Real, 0.0);
        i32 section_idx = -1;
        Status fric_status = Status::ok;
        const FrictionQuery<Real> fquery{tire_mu, geometry.point[0], slip};
        FrictionResult<Real> fresult{};
        if (!DistributedIoTraits<Real, Io>::custom_friction(
                io, fquery, fresult, fric_status, local_err)) {
            st = effective_friction<Real, TK>(
                tire_mu, &model->surface_map, geometry.point[0], slip,
                mu, section_idx, local_err);
            if (st != Status::ok) {
                err = local_err;
                return st;
            }
        } else if (fric_status != Status::ok) {
            err = local_err;
            return fric_status;
        } else {
            mu = fresult.mu;
            section_idx = fresult.section_idx;
        }

        const Real v_roll = TK::dot3(center_velocity, tangent);
        Real xi_new = BIKE_REAL(Real, 0.0);
        Real tangent_force = BIKE_REAL(Real, 0.0);
        Real brush_loss = BIKE_REAL(Real, 0.0);
        st = brush_step<Real, TK>(
            xi, slip, v_roll, load, k_station, mu, relax_len, dt,
            xi_new, tangent_force, brush_loss, local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }
        losses += brush_loss;
        shear_energy += BIKE_REAL(Real, 0.5) * k_station * TK::pow2(xi_new);

        const Real force_world[3] = {
            load * normal[0] + tangent_force * tangent[0],
            load * normal[1] + tangent_force * tangent[1],
            load * normal[2] + tangent_force * tangent[2]
        };
        TK::gemv_t3(nv, ws.jac_station, force_world, ws.tmp);
        for (i32 i = 0; i < nv; ++i) {
            out_qfrc[i] += ws.tmp[i];
        }

        const DistributedStationRecord<Real> srec{
            j,
            sid,
            TyreVec3<Real>{point[0], point[1], point[2]},
            TyreVec3<Real>{normal[0], normal[1], normal[2]},
            geometry,
            load,
            elastic_energy,
            penetration,
            tangent_force,
            slip
        };
        st = DistributedIoTraits<Real, Io>::record_station(
            io, side, srec, local_err);
        if (st != Status::ok) {
            err = local_err;
            return st;
        }

        next_states[sid].active = 1;
        next_states[sid].xi = xi_new;
        next_states[sid].tangent[0] = tangent[0];
        next_states[sid].tangent[1] = tangent[1];
        next_states[sid].tangent[2] = tangent[2];
        next_states[sid].has_tangent = 1;
    }

    i32 next_active_j = 0;
    for (i32 i = 0; i < station_count; ++i) {
        if (next_active_j < count && ws.active_ids[next_active_j] == i) {
            ++next_active_j;
            continue;
        }
        if (prev_states[i].active != 0) {
            const Real w_i = weights != nullptr ? weights[i] : uniform_weight;
            losses += BIKE_REAL(Real, 0.5) * tk * w_i *
                      TK::pow2(prev_states[i].xi);
        }
        next_states[i].active = 0;
        next_states[i].xi = BIKE_REAL(Real, 0.0);
        next_states[i].tangent[0] = BIKE_REAL(Real, 0.0);
        next_states[i].tangent[1] = BIKE_REAL(Real, 0.0);
        next_states[i].tangent[2] = BIKE_REAL(Real, 0.0);
        next_states[i].has_tangent = 0;
    }

    const DistributedWheelRecord<Real> wrec{
        center_vec,
        count,
        losses,
        shear_energy,
        radial_power
    };
    st = DistributedIoTraits<Real, Io>::record_wheel(
        io, side, wrec, local_err);
    if (st != Status::ok) {
        err = local_err;
        return st;
    }

    err = ErrCode::none;
    return Status::ok;
}

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
