// native/src/gpu/tire_kernel.hpp — Allocation-free __host__ __device__ distributed
// tire and 3D multi-rib ray-grid contact kernel for C++23 and CUDA/nvcc.
#pragma once

#include "common.hpp"

namespace bike::gpu {

inline constexpr float kTwoPi = 6.28318530717958647692f;
inline constexpr float kMaxRibCrownAngleRad = 1.04719755119659774615f; // pi / 3

struct GpuHingeDensity {
    int num_knots = 3;
    float knots_m[kMaxHingeKnots] = {
        0.0f, 0.008f, 0.018f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    float stiffness_n_m[kMaxHingeKnots] = {
        55000.0f, 85000.0f, 140000.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };
    float damping_ns_m = 420.0f;
};

struct GpuTireParams {
    float nominal_radius_m = 0.351f;
    float section_radius_m = 0.030f;
    int num_stations = 32;
    int num_lateral_ribs = 1;
    GpuHingeDensity density{};
    float bristle_kx_n_m = 14000.0f;
    float bristle_cx_ns_m = 35.0f;
    float mu_static = 0.95f;
    float mu_kinetic = 0.85f;
    float relaxation_length_m = 0.018f;
};

struct GpuHeightfield1D {
    const float *heights_m = nullptr;
    int num_samples = 0;
    float x_min_m = 0.0f;
    float dx_m = 1.0f;
    float inv_dx_m = 1.0f;
};

struct GpuWheelStationState {
    float bristle_x[kMaxTreadStations] = {};
};

struct GpuWheelContactOutput {
    float fx_world_n = 0.0f;
    float fz_world_n = 0.0f;
    float spin_torque_nm = 0.0f;
    float normal_load_n = 0.0f;
    float effective_radius_m = 0.0f;
    int active_stations = 0;
};

// Mirrors biketyre::station_density in native/src/tyre/station.cpp:52-73 verbatim.
// Returns GpuVec2{elastic_force, elastic_energy}.
BIKE_HD GpuVec2 gpu_station_density(
    float delta_m,
    const GpuHingeDensity &d
) noexcept {
    float force = 0.0f;
    float energy = 0.0f;
    const int knots = d.num_knots < 0
        ? 0
        : (d.num_knots > kMaxHingeKnots ? kMaxHingeKnots : d.num_knots);
    for (int i = 0; i < knots; ++i) {
        const float x = gpu_max(delta_m - d.knots_m[i], 0.0f);
        force += d.stiffness_n_m[i] * x;
        energy += d.stiffness_n_m[i] * x * x;
    }
    return {force, 0.5f * energy};
}

// Mirrors biketyre::station_response in native/src/tyre/station.cpp:78-95 verbatim.
// Returns GpuVec2{force, energy}.
BIKE_HD GpuVec2 gpu_station_response(
    float delta_m,
    float rate_mps,
    float weight,
    const GpuHingeDensity &d
) noexcept {
    const GpuVec2 dens = gpu_station_density(delta_m, d);
    const float force = delta_m > 0.0f
        ? gpu_max(0.0f, dens.x + d.damping_ns_m * rate_mps)
        : 0.0f;
    return {weight * force, weight * dens.y};
}

// Samples a 1D heightfield via clamped linear interpolation and returns
// GpuVec3{height_z, normal_x, normal_z} with unit normal (-dh/dx, 1) / sqrt(1 + (dh/dx)^2).
BIKE_HD GpuVec3 gpu_sample_heightfield_1d(
    const GpuHeightfield1D &hf,
    float x_m
) noexcept {
    if (hf.heights_m == nullptr || hf.num_samples <= 0) {
        return {0.0f, 0.0f, 1.0f};
    }
    if (hf.num_samples == 1) {
        return {hf.heights_m[0], 0.0f, 1.0f};
    }
    const float inv_dx = hf.inv_dx_m > 0.0f
        ? hf.inv_dx_m
        : (hf.dx_m > 0.0f ? (1.0f / hf.dx_m) : 1.0f);
    const float max_coord = static_cast<float>(hf.num_samples - 1);
    const float u = gpu_clamp((x_m - hf.x_min_m) * inv_dx, 0.0f, max_coord);
    int i0 = static_cast<int>(u);
    if (i0 > hf.num_samples - 2) {
        i0 = hf.num_samples - 2;
    }
    if (i0 < 0) {
        i0 = 0;
    }
    const float t = gpu_clamp(u - static_cast<float>(i0), 0.0f, 1.0f);
    const float h0 = hf.heights_m[i0];
    const float h1 = hf.heights_m[i0 + 1];
    const float height_z = h0 + t * (h1 - h0);
    const float dh_dx = (h1 - h0) * inv_dx;
    const float inv_norm = 1.0f / gpu_sqrt(1.0f + dh_dx * dh_dx);
    const float normal_x = -dh_dx * inv_norm;
    const float normal_z = inv_norm;
    return {height_z, normal_x, normal_z};
}

// Evaluates distributed tire contact over num_stations (and optional num_lateral_ribs
// for 3D carcass ray scaling), updates tangential bristle state when num_lateral_ribs == 1,
// and accumulates world forces, spin torque, normal load, and load-weighted effective radius.
BIKE_HD GpuWheelContactOutput gpu_evaluate_wheel_contact(
    float wheel_x_m,
    float wheel_z_m,
    float wheel_vx_mps,
    float wheel_vz_mps,
    float wheel_omega_rad_s,
    float dt_s,
    const GpuTireParams &params,
    const GpuHeightfield1D &hf,
    GpuWheelStationState *station_state = nullptr
) noexcept {
    GpuWheelContactOutput out{};
    const int n_stations = params.num_stations > 0 ? params.num_stations : 32;
    const int n_ribs = params.num_lateral_ribs > 0 ? params.num_lateral_ribs : 1;
    const float station_weight = kTwoPi / static_cast<float>(n_stations);
    const float rib_weight = station_weight / static_cast<float>(n_ribs);
    const float safe_dt = gpu_max(dt_s, 0.0f);
    const float sigma = gpu_max(params.relaxation_length_m, 1e-4f);

    float weighted_drop_terms = 0.0f;
    float weighted_drop_weights = 0.0f;
    float unweighted_drop_terms = 0.0f;
    int unweighted_drop_count = 0;

    for (int s = 0; s < n_stations; ++s) {
        const float theta = kTwoPi * static_cast<float>(s) / static_cast<float>(n_stations);
        const float cos_theta = gpu_cos(theta);
        const float sin_theta = gpu_sin(theta);

        for (int rib = 0; rib < n_ribs; ++rib) {
            float r_rib = params.nominal_radius_m;
            if (n_ribs > 1) {
                const float u_lat = (2.0f * static_cast<float>(rib) /
                                     static_cast<float>(n_ribs - 1)) - 1.0f;
                const float psi = u_lat * kMaxRibCrownAngleRad;
                r_rib = params.nominal_radius_m -
                        params.section_radius_m * (1.0f - gpu_cos(psi));
            }

            const float rx = r_rib * cos_theta;
            const float rz = r_rib * sin_theta;
            const float pt_x = wheel_x_m + rx;
            const float pt_z = wheel_z_m + rz;

            const GpuVec3 surf = gpu_sample_heightfield_1d(hf, pt_x);
            const float terrain_z = surf.x;
            const float nx = surf.y;
            const float nz = surf.z;

            // Signed vertical penetration projected onto the surface normal.
            const float delta_m = (terrain_z - pt_z) * nz;
            if (delta_m <= 0.0f) {
                if (n_ribs == 1 && s < kMaxTreadStations && station_state != nullptr) {
                    station_state->bristle_x[s] = 0.0f;
                }
                continue;
            }

            // Station point world velocity: v_center + omega_y x r_station
            // In X-Z plane with +Y spin axis: v_x = vx + omega * rz, v_z = vz - omega * rx
            const float vx_pt = wheel_vx_mps + wheel_omega_rad_s * rz;
            const float vz_pt = wheel_vz_mps - wheel_omega_rad_s * rx;

            // Penetration rate = -(v_pt . n)
            const float rate_mps = -(vx_pt * nx + vz_pt * nz);
            const GpuVec2 resp = gpu_station_response(
                delta_m, rate_mps, rib_weight, params.density);
            const float normal_force = resp.x;

            // Forward unit tangent in X-Z plane: t = (nz, -nx)
            const float tx = nz;
            const float tz = -nx;
            const float v_slip = vx_pt * tx + vz_pt * tz;
            const float v_trans = gpu_abs(wheel_vx_mps * tx + wheel_vz_mps * tz);

            float xi_prev = 0.0f;
            if (n_ribs == 1 && s < kMaxTreadStations && station_state != nullptr) {
                xi_prev = station_state->bristle_x[s];
            }

            const float decay = 1.0f / (1.0f + (v_trans * safe_dt) / sigma);
            float xi_new = (xi_prev - v_slip * safe_dt) * decay;

            const float k_station = params.bristle_kx_n_m * rib_weight;
            const float c_station = params.bristle_cx_ns_m * rib_weight;
            const float ft_raw = k_station * xi_new - c_station * v_slip;

            const float slip_blend = gpu_min(gpu_abs(v_slip) / 0.5f, 1.0f);
            const float mu_eff = params.mu_kinetic +
                (params.mu_static - params.mu_kinetic) * (1.0f - slip_blend);
            const float max_ft = gpu_max(0.0f, mu_eff * normal_force);
            const float tangent_force = gpu_clamp(ft_raw, -max_ft, max_ft);

            if (k_station > 1e-8f) {
                const float max_xi = max_ft / k_station;
                xi_new = gpu_clamp(xi_new, -max_xi, max_xi);
            }
            if (n_ribs == 1 && s < kMaxTreadStations && station_state != nullptr) {
                station_state->bristle_x[s] = xi_new;
            }

            const float fx_station = normal_force * nx + tangent_force * tx;
            const float fz_station = normal_force * nz + tangent_force * tz;

            out.fx_world_n += fx_station;
            out.fz_world_n += fz_station;
            out.spin_torque_nm += rz * fx_station - rx * fz_station;
            out.normal_load_n += normal_force;
            out.active_stations += 1;

            // Load-weighted radial drop matching effective_radius in distributed_tire.cpp:135-155
            const float drop = -(rx * nx + rz * nz);
            if (normal_force > 0.0f) {
                weighted_drop_terms += normal_force * drop;
                weighted_drop_weights += normal_force;
            }
            unweighted_drop_terms += drop;
            unweighted_drop_count += 1;
        }
    }

    if (out.active_stations > 0) {
        if (weighted_drop_weights > 0.0f) {
            out.effective_radius_m = gpu_max(0.0f, weighted_drop_terms / weighted_drop_weights);
        } else if (unweighted_drop_count > 0) {
            out.effective_radius_m = gpu_max(
                0.0f, unweighted_drop_terms / static_cast<float>(unweighted_drop_count));
        }
    }
    return out;
}

// Overload with (params, hf, station_state, dt_s) parameter ordering for caller convenience.
BIKE_HD GpuWheelContactOutput gpu_evaluate_wheel_contact(
    float wheel_x_m,
    float wheel_z_m,
    float wheel_vx_mps,
    float wheel_vz_mps,
    float wheel_omega_rad_s,
    const GpuTireParams &params,
    const GpuHeightfield1D &hf,
    GpuWheelStationState *station_state = nullptr,
    float dt_s = 0.001f
) noexcept {
    return gpu_evaluate_wheel_contact(
        wheel_x_m,
        wheel_z_m,
        wheel_vx_mps,
        wheel_vz_mps,
        wheel_omega_rad_s,
        dt_s,
        params,
        hf,
        station_state
    );
}

// Evaluates front and rear wheels and projects their forces onto qfrc_world
// (root_x_dof, root_z_dof, root_pitch_dof, front_wheel_spin_dof, rear_wheel_spin_dof).
BIKE_HD void gpu_write_both_wheels_qfrc(
    float com_x_m,
    float com_z_m,
    float front_x_m,
    float front_z_m,
    float front_vx_mps,
    float front_vz_mps,
    float front_omega_rad_s,
    float rear_x_m,
    float rear_z_m,
    float rear_vx_mps,
    float rear_vz_mps,
    float rear_omega_rad_s,
    float dt_s,
    const GpuTireParams &front_params,
    const GpuTireParams &rear_params,
    const GpuHeightfield1D &hf,
    const GpuModelIndices &idx,
    GpuWheelStationState *front_state,
    GpuWheelStationState *rear_state,
    float *qfrc_world,
    GpuWheelContactOutput *front_out_opt = nullptr,
    GpuWheelContactOutput *rear_out_opt = nullptr
) noexcept {
    const GpuWheelContactOutput front_out = gpu_evaluate_wheel_contact(
        front_x_m,
        front_z_m,
        front_vx_mps,
        front_vz_mps,
        front_omega_rad_s,
        dt_s,
        front_params,
        hf,
        front_state
    );
    const GpuWheelContactOutput rear_out = gpu_evaluate_wheel_contact(
        rear_x_m,
        rear_z_m,
        rear_vx_mps,
        rear_vz_mps,
        rear_omega_rad_s,
        dt_s,
        rear_params,
        hf,
        rear_state
    );

    if (qfrc_world != nullptr) {
        project_planar_force_to_root(
            front_out.fx_world_n,
            front_out.fz_world_n,
            front_x_m,
            front_z_m,
            com_x_m,
            com_z_m,
            idx,
            qfrc_world
        );
        project_planar_force_to_root(
            rear_out.fx_world_n,
            rear_out.fz_world_n,
            rear_x_m,
            rear_z_m,
            com_x_m,
            com_z_m,
            idx,
            qfrc_world
        );
        qfrc_world[idx.front_wheel_spin_dof] += front_out.spin_torque_nm;
        qfrc_world[idx.rear_wheel_spin_dof] += rear_out.spin_torque_nm;
    }

    if (front_out_opt != nullptr) {
        *front_out_opt = front_out;
    }
    if (rear_out_opt != nullptr) {
        *rear_out_opt = rear_out;
    }
}

// Overload accepting body xpos [nbody * 3] and generalized velocity qvel [nv] arrays.
BIKE_HD void gpu_write_both_wheels_qfrc(
    const float *xpos,
    const float *qvel,
    float dt_s,
    const GpuTireParams &front_params,
    const GpuTireParams &rear_params,
    const GpuHeightfield1D &hf,
    const GpuModelIndices &idx,
    GpuWheelStationState *front_state,
    GpuWheelStationState *rear_state,
    float *qfrc_world,
    GpuWheelContactOutput *front_out_opt = nullptr,
    GpuWheelContactOutput *rear_out_opt = nullptr
) noexcept {
    const float com_x = xpos[3 * idx.frame_bid + 0];
    const float com_z = xpos[3 * idx.frame_bid + 2];
    const float front_x = xpos[3 * idx.front_wheel_bid + 0];
    const float front_z = xpos[3 * idx.front_wheel_bid + 2];
    const float rear_x = xpos[3 * idx.rear_wheel_bid + 0];
    const float rear_z = xpos[3 * idx.rear_wheel_bid + 2];

    const float root_vx = qvel[idx.root_x_dof];
    const float root_vz = qvel[idx.root_z_dof];
    const float pitch_rate = qvel[idx.root_pitch_dof];

    const float front_vx = root_vx + pitch_rate * (front_z - com_z);
    const float front_vz = root_vz - pitch_rate * (front_x - com_x);
    const float front_omega = pitch_rate + qvel[idx.front_wheel_spin_dof];

    const float rear_vx = root_vx + pitch_rate * (rear_z - com_z);
    const float rear_vz = root_vz - pitch_rate * (rear_x - com_x);
    const float rear_omega = pitch_rate + qvel[idx.rear_wheel_spin_dof];

    gpu_write_both_wheels_qfrc(
        com_x,
        com_z,
        front_x,
        front_z,
        front_vx,
        front_vz,
        front_omega,
        rear_x,
        rear_z,
        rear_vx,
        rear_vz,
        rear_omega,
        dt_s,
        front_params,
        rear_params,
        hf,
        idx,
        front_state,
        rear_state,
        qfrc_world,
        front_out_opt,
        rear_out_opt
    );
}

} // namespace bike::gpu
