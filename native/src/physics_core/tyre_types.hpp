// native/src/physics_core/tyre_types.hpp — Tri-target POD views, model specs,
// and persistent wheel/station states shared by CPU FP64 and GPU FP32 kernels.
#pragma once

#include "common.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters) shared host/CUDA/MSL POD types: MSL forbids std::array/std::span and double tokens. Register: docs/TESTING.md GPU kernel row.
BIKE_GPU_SHARED_CODE_BEGIN

namespace bike::physics_core {

// Exact MuJoCo mjtJoint integer codes (mjJNT_SLIDE == 2, mjJNT_HINGE == 3).
BIKE_CONSTEXPR i32 kJntSlide = 2;
BIKE_CONSTEXPR i32 kJntHinge = 3;

enum class JointKind : i32 {
    slide = kJntSlide,
    hinge = kJntHinge
};

struct GpuChainJoint {
    i32 joint_id = -1;
    i32 joint_type = 0;
    i32 dof_adr = -1;
};

struct GpuWheelChain {
    i32 wheel_body_id = -1;
    i32 njnt = 0;
    i32 joint_count = 0;
    GpuChainJoint joints[kMaxChainJoints]{};
};

template <class Real>
struct PolylineView {
    BIKE_DEVICE const Real *px = nullptr;
    BIKE_DEVICE const Real *pz = nullptr;
    BIKE_DEVICE const Real *seg_x = nullptr;
    BIKE_DEVICE const Real *seg_z = nullptr;
    BIKE_DEVICE const Real *seg_len_sq = nullptr;
    BIKE_DEVICE const i32 *height_changes = nullptr;
    BIKE_DEVICE const Real *left_x = nullptr;
    BIKE_DEVICE const Real *left_z = nullptr;
    BIKE_DEVICE const Real *right_x = nullptr;
    BIKE_DEVICE const Real *right_z = nullptr;
    i32 num_vertices = 0;
    Real maximum_z = static_cast<Real>(0);
    Real significant_delta_m = static_cast<Real>(0);
    Real significance_fraction = static_cast<Real>(0);
    Real normal_cosine = static_cast<Real>(0);
};

template <class Real>
struct PolylineContactResult {
    Real point[2]{};
    Real normal[2]{};
    Real delta = static_cast<Real>(0);
    i32 segment_id = -1;
    u8 multi_support = 0;
};

template <class Real>
struct SignedProfileView {
    BIKE_DEVICE const Real *x = nullptr;
    BIKE_DEVICE const Real *z = nullptr;
    BIKE_DEVICE const Real *edge_x = nullptr;
    BIKE_DEVICE const Real *edge_z = nullptr;
    BIKE_DEVICE const Real *length2 = nullptr;
    BIKE_DEVICE const Real *normal_x = nullptr;
    BIKE_DEVICE const Real *normal_z = nullptr;
    i32 num_vertices = 0;
};

template <class Real>
struct PointDistanceResult {
    Real distance = static_cast<Real>(0);
    Real normal[2]{};
    Real point[2]{};
    i32 segment = -1;
    u8 nonsmooth = 0;
};

template <class Real>
struct StationRingView {
    BIKE_DEVICE const Real *local_xyz = nullptr;
    BIKE_DEVICE const Real *weights = nullptr;
    Real uniform_weight = static_cast<Real>(0);
    i32 station_count = 0;
};

template <class Real>
struct HingeDensityView {
    Real knots_m[kMaxHingeKnots]{};
    Real stiffness_n_m[kMaxHingeKnots]{};
    i32 knot_count = 0;
    Real damping_ns_m = static_cast<Real>(0);
    Real loss_factor = BIKE_REAL(Real, 0.05);
    Real hysteresis_rate_eps_mps = BIKE_REAL(Real, 0.01);
};

template <class Real>
struct CompliantMaterialView {
    Real radial_k_n_m = static_cast<Real>(0);
    Real loss_factor = static_cast<Real>(0);
    Real hysteresis_rate_eps_mps = static_cast<Real>(0);
    Real valid_load_min_n = static_cast<Real>(0);
    Real valid_load_max_n = static_cast<Real>(0);
    u8 has_tabulated_radial = 0;
};

template <class Real>
struct SurfaceSpecPod {
    Real mu_peak = static_cast<Real>(0);
    Real mu_slide = static_cast<Real>(0);
    Real slip_stiffness_per_load = static_cast<Real>(0);
    Real stribeck_speed_mps = static_cast<Real>(0);
    i32 surface_id = 0;
};

template <class Real>
struct SurfaceSectionView {
    Real start_m = static_cast<Real>(0);
    Real end_m = static_cast<Real>(0);
    SurfaceSpecPod<Real> surface{};
};

template <class Real>
struct SurfaceMapView {
    SurfaceSpecPod<Real> default_surface{};
    SurfaceSectionView<Real> sections[kMaxSurfaceSections]{};
    i32 section_count = 0;
    u8 has_surface_map = 0;
    u8 pad[3]{};
};

enum class TyreBackendKind : i32 {
    compliant_2d = 0,
    distributed_2d_reference = 1
};

template <class Real>
struct TyreWheelSpec {
    GpuWheelChain chain{};
    i32 geom_id = -1;
    Real radius_m = static_cast<Real>(0);
    Real tangent_k_n_m = static_cast<Real>(0);
    Real tire_mu = static_cast<Real>(0);
    Real relaxation_length_m = static_cast<Real>(0);
    CompliantMaterialView<Real> compliant{};
    HingeDensityView<Real> distributed_density{};
    Real valid_deflection_min_m = static_cast<Real>(0);
    Real valid_deflection_max_m = static_cast<Real>(0);
    Real local_xyz[3 * kMaxGpuTyreStations]{};
    Real station_weight = static_cast<Real>(0);
};

template <class Real>
struct TyreModel {
    TyreBackendKind backend = TyreBackendKind::compliant_2d;
    i32 nq = 0;
    i32 nv = 0;
    i32 nbody = 0;
    i32 njnt = 0;
    i32 station_count = 0;
    Real distinct_normal_cosine = static_cast<Real>(0);
    TyreWheelSpec<Real> wheels[2]{};
    SurfaceMapView<Real> surface_map{};
};

template <class Real>
struct CompliantWheelStatePod {
    Real xi = static_cast<Real>(0);
    Real tangent[3]{};
    Real point[3]{};
    Real center[3]{};
    i32 segment = -1;
    u8 has_tangent = 0;
    u8 has_point = 0;
    u8 has_center = 0;
    u8 has_segment = 0;
};

template <class Real>
struct StationStatePod {
    Real xi = static_cast<Real>(0);
    Real tangent[3]{};
    u8 active = 0;
    u8 has_tangent = 0;
    u8 pad[2]{};
};

using GpuTyreModel = TyreModel<float>;
using GpuCompliantWheelState = CompliantWheelStatePod<float>;
using GpuStationState = StationStatePod<float>;

} // namespace bike::physics_core

BIKE_GPU_SHARED_CODE_END
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays,bugprone-easily-swappable-parameters)
