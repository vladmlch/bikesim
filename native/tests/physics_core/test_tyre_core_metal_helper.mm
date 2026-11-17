#include "test_tyre_core_metal_helper.hpp"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <array>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace bike::physics_core::test_tyre_metal {

namespace {

std::string read_file_str(const std::string &path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        return "";
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string strip_includes_and_pragmas(const std::string &src) {
    std::istringstream iss(src);
    std::ostringstream oss;
    std::string line;
    while (std::getline(iss, line)) {
        const std::size_t first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos) {
            if (line.compare(first, 12, "#pragma once") == 0 ||
                line.compare(first, 8, "#include") == 0) {
                continue;
            }
        }
        oss << line << '\n';
    }
    return oss.str();
}

std::string build_msl_source(
    const std::string &physics_core_dir,
    bool include_distributed,
    std::string &err_out
) {
    std::vector<const char *> headers = {
        "common.hpp",
        "gpu_math.hpp",
        "gpu_toolkit.hpp",
        "tyre_types.hpp",
        "tyre_contact.hpp",
        "tyre_compliant.hpp"
    };
    if (include_distributed) {
        headers.push_back("tyre_distributed.hpp");
    }

    std::string combined = R"MSL(
#pragma METAL fp contract(off)
#pragma clang diagnostic ignored "-Wc++17-extensions"
#include <metal_stdlib>
using namespace metal;
)MSL";

    for (const char *name : headers) {
        const std::string path = physics_core_dir + "/" + name;
        const std::string raw = read_file_str(path);
        if (raw.empty()) {
            err_out = "Failed to read header: " + path;
            return "";
        }
        combined += "\n// === " + std::string(name) + " ===\n";
        combined += strip_includes_and_pragmas(raw);
    }

    combined += R"MSL(
struct SmokeTerrainFn {
    bike::physics_core::Status operator()(
        float cx,
        float cz,
        float radius,
        bool,
        bike::physics_core::i32,
        thread bike::physics_core::PolylineContactResult<float> &out,
        thread bike::physics_core::ErrCode &err
    ) const {
        out.point[0] = cx;
        out.point[1] = 0.0f;
        out.normal[0] = 0.0f;
        out.normal[1] = 1.0f;
        out.delta = radius - cz;
        out.segment_id = 0;
        out.multi_support = 0;
        err = bike::physics_core::ErrCode::none;
        return bike::physics_core::Status::ok;
    }
};

struct SmokeJacFn {
    float jac[9];

    bike::physics_core::Status operator()(
        bike::physics_core::i32,
        bike::physics_core::CompliantJacInput<float> in_j,
        thread const float *qvel,
        thread bike::physics_core::CompliantJacOutput<float> &out_j,
        thread bike::physics_core::ErrCode &err
    ) {
        for (int i = 0; i < 9; ++i) {
            jac[i] = 0.0f;
        }
        // q = [x, z, pitch]
        jac[0 * 3 + 0] = 1.0f;
        jac[0 * 3 + 2] = in_j.point.z - in_j.center.z;
        jac[2 * 3 + 1] = 1.0f;
        jac[2 * 3 + 2] = -(in_j.point.x - in_j.center.x);
        float vel_arr[3] = {0.0f, 0.0f, 0.0f};
        bike::physics_core::GpuToolkit::gemv_n3(3, jac, qvel, vel_arr);
        out_j.velocity = {vel_arr[0], vel_arr[1], vel_arr[2]};
        out_j.center_velocity = {qvel[0], 0.0f, qvel[1]};
        err = bike::physics_core::ErrCode::none;
        return bike::physics_core::Status::ok;
    }
};

struct SmokeAccumFn {
    thread const float *jac;

    bike::physics_core::Status operator()(
        bike::physics_core::i32,
        bike::physics_core::TyreVec3<float>,
        bike::physics_core::TyreVec3<float> force_world,
        thread float *out_qfrc,
        thread bike::physics_core::ErrCode &err
    ) const {
        float fw[3] = {force_world.x, force_world.y, force_world.z};
        float tmp[3];
        bike::physics_core::GpuToolkit::gemv_t3(3, jac, fw, tmp);
        for (int i = 0; i < 3; ++i) {
            out_qfrc[i] += tmp[i];
        }
        err = bike::physics_core::ErrCode::none;
        return bike::physics_core::Status::ok;
    }
};

struct SmokeOut {
    float compliant_xi;
    float compliant_qfrc0;
    float distributed_xi0;
    float distributed_qfrc0;
};

kernel void tyre_core_smoke_kernel(
    constant bike::physics_core::TyreModel<float> &model [[buffer(0)]],
    device SmokeOut *out [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    if (gid != 0u) {
        return;
    }
    using namespace bike::physics_core;
    float center[3] = {0.5f, 0.0f, 0.34f};
    float qvel[3] = {2.5f, -0.02f, 7.0f};
    float qfrc[3] = {0.0f, 0.0f, 0.0f};
    CompliantWheelStatePod<float> st_prev{};
    CompliantWheelStatePod<float> st_next{};
    SmokeTerrainFn terr{};
    SmokeJacFn jac{};
    SmokeAccumFn accum{jac.jac};
    NoOpIo io{};
    ErrCode err = ErrCode::none;
    step_compliant_wheel<float, GpuToolkit>(
        &model, 0, 0.00125f, center, qvel, &st_prev, &st_next, qfrc,
        terr, jac, accum, io, err);
    out->compliant_xi = st_next.xi;
    out->compliant_qfrc0 = qfrc[0];
    out->distributed_xi0 = 0.0f;
    out->distributed_qfrc0 = 0.0f;
}
)MSL";

    if (include_distributed) {
        combined += R"MSL(
struct SmokeDistHeightFn {
    bike::physics_core::Status operator()(
        float,
        thread float &out_h,
        thread bike::physics_core::ErrCode &err
    ) const {
        out_h = 0.0f;
        err = bike::physics_core::ErrCode::none;
        return bike::physics_core::Status::ok;
    }
};

struct SmokeDistQueryFn {
    bike::physics_core::Status operator()(
        bike::physics_core::TyreXZ<float> xz,
        thread bike::physics_core::PointDistanceResult<float> &out,
        thread bike::physics_core::ErrCode &err
    ) const {
        out.distance = xz.z;
        out.normal[0] = 0.0f;
        out.normal[1] = 1.0f;
        out.point[0] = xz.x;
        out.point[1] = 0.0f;
        out.segment = 0;
        out.nonsmooth = 0;
        err = bike::physics_core::ErrCode::none;
        return bike::physics_core::Status::ok;
    }
};

struct SmokeCenterJacFn {
    bike::physics_core::Status operator()(
        bike::physics_core::TyreVec3<float>,
        thread float *jp_center,
        thread float *jr,
        thread bike::physics_core::ErrCode &err
    ) const {
        for (int i = 0; i < 9; ++i) {
            jp_center[i] = 0.0f;
            jr[i] = 0.0f;
        }
        jp_center[0 * 3 + 0] = 1.0f;
        jp_center[2 * 3 + 1] = 1.0f;
        jr[1 * 3 + 2] = 1.0f;
        err = bike::physics_core::ErrCode::none;
        return bike::physics_core::Status::ok;
    }
};

kernel void tyre_core_dist_smoke_kernel(
    constant bike::physics_core::TyreModel<float> &model [[buffer(0)]],
    device bike::physics_core::StationStatePod<float> *prev_states [[buffer(1)]],
    device bike::physics_core::StationStatePod<float> *next_states [[buffer(2)]],
    device SmokeOut *out [[buffer(3)]],
    uint gid [[thread_position_in_grid]]
) {
    if (gid != 0u) {
        return;
    }
    using namespace bike::physics_core;
    float center[3] = {0.5f, 0.0f, 0.34f};
    float xmat[9] = {1.0f, 0.0f, 0.0f,
                     0.0f, 1.0f, 0.0f,
                     0.0f, 0.0f, 1.0f};
    float qvel[3] = {2.5f, -0.02f, 7.0f};
    float qfrc[3] = {0.0f, 0.0f, 0.0f};
    GpuDistributedWorkspace<float, 128u, 3u> ws{};
    SmokeDistHeightFn hfn{};
    SmokeDistQueryFn qfn{};
    SmokeCenterJacFn jfn{};
    NoOpIo io{};
    ErrCode err = ErrCode::none;
    step_distributed_wheel<float, GpuToolkit>(
        &model, 0, 0.00125f, center, xmat, qvel,
        model.wheels[0].local_xyz,
        static_cast<thread const float *>(nullptr),
        prev_states, next_states, qfrc, ws, hfn, qfn, jfn, io, err);
    out->distributed_xi0 = next_states[0].xi;
    out->distributed_qfrc0 = qfrc[0];
}
)MSL";
    }

    return combined;
}

} // namespace

MetalTyreCompileResult run_metal_tyre_smoke(
    const std::string &physics_core_dir,
    bool include_distributed
) {
    MetalTyreCompileResult res;
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) {
            return res;
        }
        res.device_available = true;
        res.device_name = [[device name] UTF8String];

        std::string src_err;
        const std::string msl_src = build_msl_source(
            physics_core_dir, include_distributed, src_err);
        if (msl_src.empty()) {
            res.error_message = src_err;
            return res;
        }

        MTLCompileOptions *opts = [[MTLCompileOptions alloc] init];
        opts.mathMode = MTLMathModeSafe;
        opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;

        NSError *err = nil;
        NSString *ns_src = [NSString stringWithUTF8String:msl_src.c_str()];
        id<MTLLibrary> library =
            [device newLibraryWithSource:ns_src options:opts error:&err];
        if (library == nil) {
            res.error_message =
                std::string("Metal MSL compile error: ") +
                (err != nil ? [[err localizedDescription] UTF8String] : "unknown");
            return res;
        }
        res.compiled = true;

        NSString *fn_name = include_distributed
            ? @"tyre_core_dist_smoke_kernel"
            : @"tyre_core_smoke_kernel";
        id<MTLFunction> fn = [library newFunctionWithName:fn_name];
        if (fn == nil) {
            res.error_message = "Missing kernel function in Metal library";
            return res;
        }
        id<MTLComputePipelineState> pso =
            [device newComputePipelineStateWithFunction:fn error:&err];
        if (pso == nil) {
            res.error_message =
                std::string("Metal pipeline error: ") +
                (err != nil ? [[err localizedDescription] UTF8String] : "unknown");
            return res;
        }
        res.executed = true;
    }
    return res;
}

} // namespace bike::physics_core::test_tyre_metal
