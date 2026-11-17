// native/src/physics_core/tyre_upload.hpp — Host-only builders and upload validation
// gates for GpuWheelChain, GpuTyreModel, PolylineView<float>, and SignedProfileView<float>.
#pragma once

#if defined(__METAL_VERSION__) || defined(__CUDA_ARCH__)
#error "tyre_upload.hpp is host-only"
#endif

#include "../config_types.hpp"
#include "common.hpp"
#include "tyre_types.hpp"

#include <mujoco/mujoco.h>

#include <vector>

namespace bike::physics_core {

struct HostTyreUploadBundle {
    GpuTyreModel gpu_model{};
    PolylineView<float> polyline_view{};
    SignedProfileView<float> signed_profile_view{};

    std::vector<float> pl_px;
    std::vector<float> pl_pz;
    std::vector<float> pl_seg_x;
    std::vector<float> pl_seg_z;
    std::vector<float> pl_seg_len_sq;
    std::vector<i32> pl_height_changes;
    std::vector<float> pl_left_x;
    std::vector<float> pl_left_z;
    std::vector<float> pl_right_x;
    std::vector<float> pl_right_z;

    std::vector<float> sp_x;
    std::vector<float> sp_z;
    std::vector<float> sp_edge_x;
    std::vector<float> sp_edge_z;
    std::vector<float> sp_length2;
    std::vector<float> sp_normal_x;
    std::vector<float> sp_normal_z;

    HostTyreUploadBundle() = default;
    HostTyreUploadBundle(const HostTyreUploadBundle &other);
    HostTyreUploadBundle(HostTyreUploadBundle &&other) noexcept;
    HostTyreUploadBundle &operator=(const HostTyreUploadBundle &other);
    HostTyreUploadBundle &operator=(HostTyreUploadBundle &&other) noexcept;
    ~HostTyreUploadBundle() = default;

    void rebind_views() noexcept;
};

[[nodiscard]] GpuWheelChain build_wheel_chain(const mjModel *m, int wheel_body_id);

[[nodiscard]] HostTyreUploadBundle build_gpu_tyre_model(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    int front_geom,
    int rear_geom
);

[[nodiscard]] HostTyreUploadBundle build_gpu_tyre_model(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    int front_geom,
    int rear_geom,
    int front_body,
    int rear_body
);

} // namespace bike::physics_core
