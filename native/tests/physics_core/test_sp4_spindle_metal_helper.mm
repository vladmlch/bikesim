#include "test_sp4_spindle_metal_helper.hpp"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace bike::physics_core::test_sp4_metal {

namespace {

// The single raw pointer -> std::span boundary of this Metal test helper:
// [MTLBuffer contents] exposes each MTLResourceStorageModeShared buffer as a
// raw void* over its allocated byte length. Register: docs/TESTING.md.
template <typename T>
[[nodiscard]] std::span<T> boundary_span(T *ptr, std::size_t count) noexcept {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    return std::span<T>(ptr, count);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
}

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

std::string build_msl_source(const std::string &physics_core_dir, std::string &err_out) {
    const std::array<const char *, 7> headers = {
        "common.hpp",
        "gpu_math.hpp",
        "gpu_toolkit.hpp",
        "launch_plan.hpp",
        "front_lift.hpp",
        "rider_intent.hpp",
        "spindle_m2k2.hpp"
    };

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
struct LegSweepScalars {
    float thigh_m;
    float link_m;
    float crank_m;
    float side_sign;
    float knee_sign;
    float hip_x;
    float hip_z;
    float det_eps;
    float reach_eps;
    uint n;
};

kernel void sp4_analytic_leg_kernel(
    constant LegSweepScalars &sc [[buffer(0)]],
    device const float *phases [[buffer(1)]],
    device bike::physics_core::LegKinematicsResult<float> *out [[buffer(2)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid >= sc.n) {
        return;
    }
    out[gid] = analytic_leg_kinematics<float, GpuToolkit>(
        sc.thigh_m, sc.link_m, sc.crank_m, sc.side_sign, sc.knee_sign,
        sc.hip_x, sc.hip_z, phases[gid], sc.det_eps, sc.reach_eps);
}

struct Sp4StateScalars {
    int n_states;
    int nC;
};

struct Sp4DenseScalars {
    int n_states;
    int qM_row_stride;
};

struct Sp4StateResultMeta {
    uchar has_min_lean;
    uchar torso_sat;
    uchar arms_sat;
    uchar pad;
    float min_lean_rad;
    float upper_targets[5];
    float f_weight[2];
    float f_weight_null_subm[2];
};

kernel void sp4_state_eval_kernel(
    constant bike::physics_core::SpindleM2K2Params<float> &params [[buffer(0)]],
    constant Sp4StateScalars &sc [[buffer(1)]],
    constant bike::physics_core::SpindleCommand<float> *cmds [[buffer(2)]],
    device const float *subtreemass [[buffer(3)]],
    device const int *M_rowadr [[buffer(4)]],
    device const int *M_rownnz [[buffer(5)]],
    device const int *M_colind [[buffer(6)]],
    device const float *qpos_all [[buffer(7)]],
    device const float *qvel_all [[buffer(8)]],
    device const float *bias_all [[buffer(9)]],
    device const float *xmat_all [[buffer(10)]],
    device const float *xpos_all [[buffer(11)]],
    device const float *xanchor_all [[buffer(12)]],
    device const float *xaxis_all [[buffer(13)]],
    device const float *site_xpos_all [[buffer(14)]],
    device const float *subtree_com_all [[buffer(15)]],
    device const float *d_M_all [[buffer(16)]],
    device const float *csr_vec_all [[buffer(17)]],
    device const float *qacc_in_all [[buffer(18)]],
    device const float *init_hold_all [[buffer(19)]],
    device const int *step_idx_all [[buffer(20)]],
    device const float *min_elbow_all [[buffer(21)]],
    device Sp4StateResultMeta *meta_out [[buffer(22)]],
    device float *csr_out_all [[buffer(23)]],
    device float *ctrl_out_all [[buffer(24)]],
    device bike::physics_core::SpindleM2K2State<float> *states_out [[buffer(25)]],
    device bike::physics_core::SpindleM2K2StepScratch<float> *scratches_out [[buffer(26)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    const int w = static_cast<int>(gid);
    if (w >= sc.n_states) {
        return;
    }

    const int nq = params.nq;
    const int nv = params.nv;
    const int nu = params.nu;
    const int nbody = params.nbody;
    const int njnt = params.njnt;
    const int nsite = params.nsite;

    device const float *qpos = qpos_all + w * nq;
    device const float *qvel = qvel_all + w * nv;
    device const float *bias = bias_all + w * nv;
    device const float *xmat = xmat_all + w * nbody * 9;
    device const float *xpos = xpos_all + w * nbody * 3;
    device const float *xanchor = xanchor_all + w * njnt * 3;
    device const float *xaxis = xaxis_all + w * njnt * 3;
    device const float *site_xpos = site_xpos_all + w * nsite * 3;
    device const float *subtree_com = subtree_com_all + w * nbody * 3;
    device const float *d_M = d_M_all + w * sc.nC;
    device const float *csr_vec = csr_vec_all + w * nv;
    device const float *qacc_p0 = qacc_in_all + (3 * w + 0) * nv;
    device const float *qacc_p1 = qacc_in_all + (3 * w + 1) * nv;
    device const float *qacc_p2 = qacc_in_all + (3 * w + 2) * nv;

    Sp4StateResultMeta meta{};
    spindle_m2k2_min_torso_lean_rad<float, GpuToolkit>(
        &params, qpos, xpos, xmat, xanchor, min_elbow_all[w],
        meta.has_min_lean, meta.min_lean_rad);

    const SpindleCommand<float> cmd = cmds[w];
    float up_targets[kNumUpperSpindleJoints]{};
    upper_targets_core<float, GpuToolkit>(
        &params, qpos, xpos, xmat, xanchor, cmd.torso_lean_rad,
        up_targets, meta.torso_sat, meta.arms_sat);
    for (int k = 0; k < kNumUpperSpindleJoints; ++k) {
        meta.upper_targets[k] = up_targets[k];
    }

    mul_m_csr<float>(nv, M_rowadr, M_rownnz, M_colind, d_M, csr_vec, csr_out_all + w * nv);

    device SpindleM2K2State<float> *st = &states_out[w];
    device SpindleM2K2StepScratch<float> *scr = &scratches_out[w];
    *st = SpindleM2K2State<float>{};
    st->coast_hold_nm = init_hold_all[w];
    *scr = SpindleM2K2StepScratch<float>{};

    float ctrl_buf[kMaxNu]{};
    for (int i = 0; i < nu; ++i) {
        ctrl_buf[i] = 0.0f;
    }

    device float *ctrl_base = ctrl_out_all + (5 * w) * nu;
    spindle_m2k2_pass0_init_and_base<float, GpuToolkit>(
        &params, cmd, step_idx_all[w],
        qpos, qvel, bias, xmat, xpos, xanchor, xaxis, site_xpos, subtree_com, subtreemass,
        M_rowadr, M_rownnz, M_colind, d_M, st, scr, ctrl_buf);
    for (int i = 0; i < nu; ++i) {
        ctrl_base[0 * nu + i] = ctrl_buf[i];
    }
    meta.f_weight[0] = scr->f_weight[0];
    meta.f_weight[1] = scr->f_weight[1];

    float fw_null[2]{};
    leg_weights_at_pedal_core<float>(
        &params, static_cast<device const float *>(nullptr),
        subtree_com, xanchor, xaxis,
        scr->gammas[0][0], scr->gammas[0][1],
        scr->gammas[1][0], scr->gammas[1][1], fw_null);
    meta.f_weight_null_subm[0] = fw_null[0];
    meta.f_weight_null_subm[1] = fw_null[1];
    meta_out[w] = meta;

    spindle_m2k2_pass1_perturb0<float, GpuToolkit>(
        &params, qpos, qvel, bias, qacc_p0,
        M_rowadr, M_rownnz, M_colind, d_M, st, scr, ctrl_buf);
    for (int i = 0; i < nu; ++i) {
        ctrl_base[1 * nu + i] = ctrl_buf[i];
    }

    spindle_m2k2_pass2_perturb1<float, GpuToolkit>(
        &params, qpos, qvel, bias, qacc_p1,
        M_rowadr, M_rownnz, M_colind, d_M, st, scr, ctrl_buf);
    for (int i = 0; i < nu; ++i) {
        ctrl_base[2 * nu + i] = ctrl_buf[i];
    }

    spindle_m2k2_pass3_commit<float, GpuToolkit>(
        &params, qpos, qvel, bias, qacc_p2,
        M_rowadr, M_rownnz, M_colind, d_M, st, scr, ctrl_buf);
    for (int i = 0; i < nu; ++i) {
        ctrl_base[3 * nu + i] = ctrl_buf[i];
    }

    float ctrl_held[kMaxNu]{};
    for (int i = 0; i < nu; ++i) {
        ctrl_held[i] = 0.0f;
    }
    spindle_m2k2_write_held_ctrl<float, GpuToolkit>(
        &params, st, qpos, qvel, st->command_enabled, ctrl_held);
    for (int i = 0; i < nu; ++i) {
        ctrl_base[4 * nu + i] = ctrl_held[i];
    }
}

kernel void sp4_torture_address_spaces_kernel(
    constant bike::physics_core::SpindleM2K2Params<float> &c_params [[buffer(0)]],
    device const bike::physics_core::SpindleM2K2Params<float> *d_params [[buffer(1)]],
    device bike::physics_core::SpindleM2K2State<float> *d_state [[buffer(2)]],
    device bike::physics_core::SpindleM2K2StepScratch<float> *d_scratch [[buffer(3)]],
    device const float *d_qpos [[buffer(4)]],
    device const float *d_qvel [[buffer(5)]],
    device const float *d_bias [[buffer(6)]],
    device const float *d_xmat [[buffer(7)]],
    device const float *d_xpos [[buffer(8)]],
    device const float *d_xanchor [[buffer(9)]],
    device const float *d_xaxis [[buffer(10)]],
    device const float *d_site_xpos [[buffer(11)]],
    device const float *d_subtree_com [[buffer(12)]],
    device const float *d_subtreemass [[buffer(13)]],
    device const int *d_M_rowadr [[buffer(14)]],
    device const int *d_M_rownnz [[buffer(15)]],
    device const int *d_M_colind [[buffer(16)]],
    device const float *d_M [[buffer(17)]],
    device const float *d_qacc [[buffer(18)]],
    device float *d_ctrl [[buffer(19)]],
    device float *out_sum [[buffer(20)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    SpindleM2K2Params<float> t_params = c_params;
    SpindleM2K2State<float> t_state{};
    SpindleM2K2StepScratch<float> t_scratch{};
    SpindleCommand<float> cmd{};
    cmd.mean_crank_torque_nm = 25.0f;
    cmd.torso_lean_rad = 0.1f;

    u8 has_ml = 0;
    float ml = 0.0f;
    spindle_m2k2_min_torso_lean_rad<float, GpuToolkit>(
        &c_params, d_qpos, d_xpos, d_xmat, d_xanchor, 0.26f, has_ml, ml);
    spindle_m2k2_min_torso_lean_rad<float, GpuToolkit>(
        d_params, d_qpos, d_xpos, d_xmat, d_xanchor, 0.26f, has_ml, ml);
    spindle_m2k2_min_torso_lean_rad<float, GpuToolkit>(
        &t_params, d_qpos, d_xpos, d_xmat, d_xanchor, 0.26f, has_ml, ml);

    spindle_m2k2_pass0_init_and_base<float, GpuToolkit>(
        &c_params, cmd, 0, d_qpos, d_qvel, d_bias, d_xmat, d_xpos, d_xanchor, d_xaxis,
        d_site_xpos, d_subtree_com, static_cast<device const float *>(nullptr),
        d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M, d_state, d_scratch, d_ctrl);
    spindle_m2k2_pass1_perturb0<float, GpuToolkit>(
        d_params, d_qpos, d_qvel, d_bias, d_qacc, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M, &t_state, d_scratch, d_ctrl);
    spindle_m2k2_pass2_perturb1<float, GpuToolkit>(
        &t_params, d_qpos, d_qvel, d_bias, d_qacc, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M, d_state, &t_scratch, d_ctrl);
    spindle_m2k2_pass3_commit<float, GpuToolkit>(
        &c_params, d_qpos, d_qvel, d_bias, d_qacc, d_M_rowadr, d_M_rownnz, d_M_colind,
        d_M, &t_state, &t_scratch, d_ctrl);
    spindle_m2k2_write_held_ctrl<float, GpuToolkit>(
        &t_params, d_state, d_qpos, d_qvel, 1u, d_ctrl);

    out_sum[0] = ml + d_ctrl[0] + t_scratch.ps0.f_passive[0];
}

kernel void sp4_dense_qm_kernel(
    constant bike::physics_core::SpindleM2K2Params<float> &params [[buffer(0)]],
    constant Sp4DenseScalars &sc [[buffer(1)]],
    constant bike::physics_core::SpindleCommand<float> *cmds [[buffer(2)]],
    device const float *qpos_all [[buffer(3)]],
    device const float *qvel_all [[buffer(4)]],
    device const float *bias_all [[buffer(5)]],
    device const float *xmat_all [[buffer(6)]],
    device const float *xpos_all [[buffer(7)]],
    device const float *xanchor_all [[buffer(8)]],
    device const float *xaxis_all [[buffer(9)]],
    device const float *site_xpos_all [[buffer(10)]],
    device const float *subtree_com_all [[buffer(11)]],
    device const float *qM_dense_all [[buffer(12)]],
    device const float *csr_vec_all [[buffer(13)]],
    device const float *qacc_in_all [[buffer(14)]],
    device const float *init_hold_all [[buffer(15)]],
    device const int *step_idx_all [[buffer(16)]],
    device float *dense_out_all [[buffer(17)]],
    device float *ctrl_dense_out_all [[buffer(18)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    const int w = static_cast<int>(gid);
    if (w >= sc.n_states) {
        return;
    }
    const int nq = params.nq;
    const int nv = params.nv;
    const int nu = params.nu;
    const int nbody = params.nbody;
    const int njnt = params.njnt;
    const int nsite = params.nsite;

    device const float *qpos = qpos_all + w * nq;
    device const float *qvel = qvel_all + w * nv;
    device const float *bias = bias_all + w * nv;
    device const float *xmat = xmat_all + w * nbody * 9;
    device const float *xpos = xpos_all + w * nbody * 3;
    device const float *xanchor = xanchor_all + w * njnt * 3;
    device const float *xaxis = xaxis_all + w * njnt * 3;
    device const float *site_xpos = site_xpos_all + w * nsite * 3;
    device const float *subtree_com = subtree_com_all + w * nbody * 3;
    device const float *qM_w = qM_dense_all + w * nv * sc.qM_row_stride;
    device const float *vec = csr_vec_all + w * nv;
    device const float *qacc_p0 = qacc_in_all + (3 * w + 0) * nv;
    device const float *qacc_p1 = qacc_in_all + (3 * w + 1) * nv;
    device const float *qacc_p2 = qacc_in_all + (3 * w + 2) * nv;

    mul_m_dense<float>(nv, sc.qM_row_stride, qM_w, vec, dense_out_all + w * nv);

    SpindleM2K2State<float> st{};
    st.coast_hold_nm = init_hold_all[w];
    SpindleM2K2StepScratch<float> scr{};
    float ctrl_buf[kMaxNu]{};
    for (int i = 0; i < nu; ++i) {
        ctrl_buf[i] = 0.0f;
    }
    spindle_m2k2_pass0_init_and_base<float, GpuToolkit>(
        &params, cmds[w], step_idx_all[w],
        qpos, qvel, bias, xmat, xpos, xanchor, xaxis, site_xpos, subtree_com,
        static_cast<device const float *>(nullptr),
        sc.qM_row_stride, nullptr, nullptr, nullptr, qM_w, &st, &scr, ctrl_buf);
    spindle_m2k2_pass1_perturb0<float, GpuToolkit>(
        &params, qpos, qvel, bias, qacc_p0,
        sc.qM_row_stride, nullptr, nullptr, nullptr, qM_w, &st, &scr, ctrl_buf);
    spindle_m2k2_pass2_perturb1<float, GpuToolkit>(
        &params, qpos, qvel, bias, qacc_p1,
        sc.qM_row_stride, nullptr, nullptr, nullptr, qM_w, &st, &scr, ctrl_buf);
    spindle_m2k2_pass3_commit<float, GpuToolkit>(
        &params, qpos, qvel, bias, qacc_p2,
        sc.qM_row_stride, nullptr, nullptr, nullptr, qM_w, &st, &scr, ctrl_buf);
    for (int i = 0; i < nu; ++i) {
        ctrl_dense_out_all[w * nu + i] = ctrl_buf[i];
    }
}
)MSL";
    return combined;
}

template <typename T>
id<MTLBuffer> make_vec_buffer(id<MTLDevice> device, const std::vector<T> &vals) {
    const std::size_t bytes = vals.size() * sizeof(T);
    if (bytes == 0) {
        return [device newBufferWithLength:4u options:MTLResourceStorageModeShared];
    }
    return [device newBufferWithBytes:vals.data()
                               length:bytes
                              options:MTLResourceStorageModeShared];
}

template <typename T>
id<MTLBuffer> make_struct_buffer(id<MTLDevice> device, const T &val) {
    return [device newBufferWithBytes:&val
                               length:sizeof(T)
                              options:MTLResourceStorageModeShared];
}

template <typename T>
id<MTLBuffer> make_empty_buffer(id<MTLDevice> device, std::size_t count) {
    const std::size_t bytes = count * sizeof(T);
    return [device newBufferWithLength:(bytes > 0 ? bytes : 4u)
                               options:MTLResourceStorageModeShared];
}

template <typename T>
std::vector<T> read_vec_buffer(id<MTLBuffer> buf, std::size_t count) {
    std::vector<T> out(count);
    if (count == 0) {
        return out;
    }
    const std::span<const T> src = boundary_span(
        static_cast<const T *>([buf contents]), count);
    std::ranges::copy(src, out.begin());
    return out;
}

struct LegSweepScalarsHost {
    float thigh_m;
    float link_m;
    float crank_m;
    float side_sign;
    float knee_sign;
    float hip_x;
    float hip_z;
    float det_eps;
    float reach_eps;
    std::uint32_t n;
};

struct Sp4StateScalarsHost {
    i32 n_states;
    i32 nC;
};

struct Sp4DenseScalarsHost {
    i32 n_states;
    i32 qM_row_stride;
};

struct Sp4StateResultMetaHost {
    u8 has_min_lean;
    u8 torso_sat;
    u8 arms_sat;
    u8 pad;
    float min_lean_rad;
    float upper_targets[5];
    float f_weight[2];
    float f_weight_null_subm[2];
};

} // namespace

std::unique_ptr<MetalSp4Output> run_metal_sp4_parity(const MetalSp4Request &req) {
    auto out = std::make_unique<MetalSp4Output>();
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) {
            out->device_available = false;
            out->error_message = "MTLCreateSystemDefaultDevice() returned nil";
            return out;
        }
        out->device_available = true;
        out->device_name = [[device name] UTF8String];

        std::string src_err;
        const std::string msl_src = build_msl_source(req.physics_core_dir, src_err);
        if (msl_src.empty()) {
            out->error_message = src_err;
            return out;
        }

        MTLCompileOptions *opts = [[MTLCompileOptions alloc] init];
        opts.mathMode = MTLMathModeSafe;
        opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;

        NSError *err = nil;
        NSString *ns_src = [NSString stringWithUTF8String:msl_src.c_str()];
        id<MTLLibrary> library = [device newLibraryWithSource:ns_src options:opts error:&err];
        if (library == nil) {
            out->error_message = std::string("Metal MSL compile error: ") +
                                 (err != nil ? [[err localizedDescription] UTF8String] : "unknown");
            return out;
        }

        // Verify MSL 2.4, 3.0, and 3.1 compilation (wp_metal_load_library < 3.2 address-space rules)
        for (const MTLLanguageVersion ver :
             {MTLLanguageVersion2_4, MTLLanguageVersion3_0, MTLLanguageVersion3_1}) {
            MTLCompileOptions *ver_opts = [[MTLCompileOptions alloc] init];
            ver_opts.mathMode = MTLMathModeSafe;
            ver_opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;
            ver_opts.languageVersion = ver;
            NSError *ver_err = nil;
            id<MTLLibrary> ver_lib = [device newLibraryWithSource:ns_src options:ver_opts error:&ver_err];
            if (ver_lib == nil) {
                out->error_message = std::string("Metal MSL 2.4/3.x compile error: ") +
                                     (ver_err != nil ? [[ver_err localizedDescription] UTF8String] : "unknown");
                return out;
            }
            id<MTLFunction> torture_fn = [ver_lib newFunctionWithName:@"sp4_torture_address_spaces_kernel"];
            if (torture_fn == nil) {
                out->error_message = "Missing sp4_torture_address_spaces_kernel in MSL library";
                return out;
            }
            id<MTLComputePipelineState> torture_pso =
                [device newComputePipelineStateWithFunction:torture_fn error:&ver_err];
            if (torture_pso == nil) {
                out->error_message = std::string("Metal MSL torture PSO error: ") +
                                     (ver_err != nil ? [[ver_err localizedDescription] UTF8String] : "unknown");
                return out;
            }
        }

        auto make_pso = [&](NSString *name) -> id<MTLComputePipelineState> {
            id<MTLFunction> fn = [library newFunctionWithName:name];
            if (fn == nil) {
                out->error_message = std::string("Missing Metal function: ") + [name UTF8String];
                return nil;
            }
            id<MTLComputePipelineState> pso =
                [device newComputePipelineStateWithFunction:fn error:&err];
            if (pso == nil) {
                out->error_message = std::string("Metal PSO error: ") +
                                     (err != nil ? [[err localizedDescription] UTF8String] : "unknown");
                return nil;
            }
            return pso;
        };

        id<MTLComputePipelineState> pso_leg = make_pso(@"sp4_analytic_leg_kernel");
        id<MTLComputePipelineState> pso_state = make_pso(@"sp4_state_eval_kernel");
        id<MTLComputePipelineState> pso_dense = make_pso(@"sp4_dense_qm_kernel");
        if (pso_leg == nil || pso_state == nil || pso_dense == nil) {
            return out;
        }

        id<MTLCommandQueue> queue = [device newCommandQueue];
        id<MTLCommandBuffer> cmd = [queue commandBuffer];
        if (queue == nil || cmd == nil) {
            out->error_message = "Metal command queue/buffer creation failed";
            return out;
        }

        // 1. Analytic leg sweep
        const std::size_t n_leg = req.leg_phases.size();
        const bool is_left = (req.leg_side == 0);
        const LegSweepScalarsHost lsc{
            .thigh_m = is_left ? req.params.thigh_m[0] : req.params.thigh_m[1],
            .link_m = is_left ? req.params.link_m[0] : req.params.link_m[1],
            .crank_m = req.params.crank_length_m,
            .side_sign = is_left ? req.params.side_sign[0] : req.params.side_sign[1],
            .knee_sign = is_left ? req.params.knee_sign[0] : req.params.knee_sign[1],
            .hip_x = req.leg_hip_x,
            .hip_z = req.leg_hip_z,
            .det_eps = req.params.det_eps,
            .reach_eps = req.params.ik_reach_eps * 10.0f,
            .n = static_cast<std::uint32_t>(n_leg),
        };
        id<MTLBuffer> b_phases = make_vec_buffer(device, req.leg_phases);
        id<MTLBuffer> b_leg_out = make_empty_buffer<LegKinematicsResult<float>>(device, n_leg);
        if (n_leg > 0) {
            id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
            [enc setComputePipelineState:pso_leg];
            [enc setBytes:&lsc length:sizeof(lsc) atIndex:0];
            [enc setBuffer:b_phases offset:0 atIndex:1];
            [enc setBuffer:b_leg_out offset:0 atIndex:2];
            const NSUInteger tg = std::min<NSUInteger>(64u, n_leg);
            [enc dispatchThreads:MTLSizeMake(n_leg, 1, 1)
                threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
            [enc endEncoding];
        }

        // 2. Multi-state eval
        const std::size_t n_st = req.states.size();
        const std::size_t nq = static_cast<std::size_t>(req.params.nq);
        const std::size_t nv = static_cast<std::size_t>(req.params.nv);
        const std::size_t nu = static_cast<std::size_t>(req.params.nu);
        const std::size_t nbody = static_cast<std::size_t>(req.params.nbody);
        const std::size_t njnt = static_cast<std::size_t>(req.params.njnt);
        const std::size_t nsite = static_cast<std::size_t>(req.params.nsite);
        const std::size_t nC = static_cast<std::size_t>(req.nC);

        std::vector<SpindleCommand<float>> cmds;
        std::vector<float> qpos_all, qvel_all, bias_all, xmat_all, xpos_all;
        std::vector<float> xanchor_all, xaxis_all, site_xpos_all, subtree_com_all;
        std::vector<float> d_M_all, csr_vec_all, qacc_in_all;
        std::vector<float> init_hold_all, min_elbow_all;
        std::vector<i32> step_idx_all;

        cmds.reserve(n_st);
        qpos_all.reserve(n_st * nq);
        qvel_all.reserve(n_st * nv);
        bias_all.reserve(n_st * nv);
        xmat_all.reserve(n_st * nbody * 9u);
        xpos_all.reserve(n_st * nbody * 3u);
        xanchor_all.reserve(n_st * njnt * 3u);
        xaxis_all.reserve(n_st * njnt * 3u);
        site_xpos_all.reserve(n_st * nsite * 3u);
        subtree_com_all.reserve(n_st * nbody * 3u);
        d_M_all.reserve(n_st * nC);
        csr_vec_all.reserve(n_st * nv);
        qacc_in_all.reserve(3u * n_st * nv);
        init_hold_all.reserve(n_st);
        min_elbow_all.reserve(n_st);
        step_idx_all.reserve(n_st);

        for (const auto &st : req.states) {
            cmds.push_back(st.cmd);
            init_hold_all.push_back(st.initial_coast_hold_nm);
            min_elbow_all.push_back(st.min_elbow_flexion_rad);
            step_idx_all.push_back(st.step_index);
            qpos_all.insert(qpos_all.end(), st.qpos.begin(), st.qpos.end());
            qvel_all.insert(qvel_all.end(), st.qvel.begin(), st.qvel.end());
            bias_all.insert(bias_all.end(), st.qfrc_bias.begin(), st.qfrc_bias.end());
            xmat_all.insert(xmat_all.end(), st.body_xmat.begin(), st.body_xmat.end());
            xpos_all.insert(xpos_all.end(), st.body_xpos.begin(), st.body_xpos.end());
            xanchor_all.insert(xanchor_all.end(), st.jnt_xanchor.begin(), st.jnt_xanchor.end());
            xaxis_all.insert(xaxis_all.end(), st.jnt_xaxis.begin(), st.jnt_xaxis.end());
            site_xpos_all.insert(site_xpos_all.end(), st.site_xpos.begin(), st.site_xpos.end());
            subtree_com_all.insert(subtree_com_all.end(), st.subtree_com.begin(), st.subtree_com.end());
            d_M_all.insert(d_M_all.end(), st.d_M.begin(), st.d_M.end());
            csr_vec_all.insert(csr_vec_all.end(), st.csr_vec.begin(), st.csr_vec.end());
            qacc_in_all.insert(qacc_in_all.end(), st.qacc_p0.begin(), st.qacc_p0.end());
            qacc_in_all.insert(qacc_in_all.end(), st.qacc_p1.begin(), st.qacc_p1.end());
            qacc_in_all.insert(qacc_in_all.end(), st.qacc_p2.begin(), st.qacc_p2.end());
        }

        const Sp4StateScalarsHost ssc{
            .n_states = static_cast<i32>(n_st),
            .nC = req.nC,
        };
        const bool do_dense = (req.qM_row_stride >= req.params.nv && n_st > 0);
        std::vector<float> qM_dense_all;
        if (do_dense) {
            const std::size_t qm_sz = nv * static_cast<std::size_t>(req.qM_row_stride);
            qM_dense_all.reserve(n_st * qm_sz);
            for (const auto &st : req.states) {
                if (st.qM_dense.size() == qm_sz) {
                    qM_dense_all.insert(qM_dense_all.end(), st.qM_dense.begin(), st.qM_dense.end());
                } else {
                    qM_dense_all.insert(qM_dense_all.end(), qm_sz, 0.0f);
                }
            }
        }
        const Sp4DenseScalarsHost dsc{
            .n_states = static_cast<i32>(n_st),
            .qM_row_stride = req.qM_row_stride,
        };

        id<MTLBuffer> b_params = make_struct_buffer(device, req.params);
        id<MTLBuffer> b_cmds = make_vec_buffer(device, cmds);
        id<MTLBuffer> b_subm = make_vec_buffer(device, req.body_subtreemass);
        id<MTLBuffer> b_radr = make_vec_buffer(device, req.M_rowadr);
        id<MTLBuffer> b_rnnz = make_vec_buffer(device, req.M_rownnz);
        id<MTLBuffer> b_cind = make_vec_buffer(device, req.M_colind);
        id<MTLBuffer> b_qpos = make_vec_buffer(device, qpos_all);
        id<MTLBuffer> b_qvel = make_vec_buffer(device, qvel_all);
        id<MTLBuffer> b_bias = make_vec_buffer(device, bias_all);
        id<MTLBuffer> b_xmat = make_vec_buffer(device, xmat_all);
        id<MTLBuffer> b_xpos = make_vec_buffer(device, xpos_all);
        id<MTLBuffer> b_xanchor = make_vec_buffer(device, xanchor_all);
        id<MTLBuffer> b_xaxis = make_vec_buffer(device, xaxis_all);
        id<MTLBuffer> b_spos = make_vec_buffer(device, site_xpos_all);
        id<MTLBuffer> b_scom = make_vec_buffer(device, subtree_com_all);
        id<MTLBuffer> b_dM = make_vec_buffer(device, d_M_all);
        id<MTLBuffer> b_cvec = make_vec_buffer(device, csr_vec_all);
        id<MTLBuffer> b_qacc_in = make_vec_buffer(device, qacc_in_all);
        id<MTLBuffer> b_ihold = make_vec_buffer(device, init_hold_all);
        id<MTLBuffer> b_sidx = make_vec_buffer(device, step_idx_all);
        id<MTLBuffer> b_melb = make_vec_buffer(device, min_elbow_all);

        id<MTLBuffer> b_meta = make_empty_buffer<Sp4StateResultMetaHost>(device, n_st);
        id<MTLBuffer> b_csrout = make_empty_buffer<float>(device, n_st * nv);
        id<MTLBuffer> b_ctrl_out = make_empty_buffer<float>(device, 5u * n_st * nu);
        id<MTLBuffer> b_st_out = make_empty_buffer<SpindleM2K2State<float>>(device, n_st);
        id<MTLBuffer> b_scr_out = make_empty_buffer<SpindleM2K2StepScratch<float>>(device, n_st);
        id<MTLBuffer> b_qM_dense = make_vec_buffer(device, qM_dense_all);
        id<MTLBuffer> b_dense_out = make_empty_buffer<float>(device, do_dense ? n_st * nv : 0u);
        id<MTLBuffer> b_ctrl_dense = make_empty_buffer<float>(device, do_dense ? n_st * nu : 0u);

        if (n_st > 0) {
            id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
            [enc setComputePipelineState:pso_state];
            [enc setBuffer:b_params offset:0 atIndex:0];
            [enc setBytes:&ssc length:sizeof(ssc) atIndex:1];
            [enc setBuffer:b_cmds offset:0 atIndex:2];
            [enc setBuffer:b_subm offset:0 atIndex:3];
            [enc setBuffer:b_radr offset:0 atIndex:4];
            [enc setBuffer:b_rnnz offset:0 atIndex:5];
            [enc setBuffer:b_cind offset:0 atIndex:6];
            [enc setBuffer:b_qpos offset:0 atIndex:7];
            [enc setBuffer:b_qvel offset:0 atIndex:8];
            [enc setBuffer:b_bias offset:0 atIndex:9];
            [enc setBuffer:b_xmat offset:0 atIndex:10];
            [enc setBuffer:b_xpos offset:0 atIndex:11];
            [enc setBuffer:b_xanchor offset:0 atIndex:12];
            [enc setBuffer:b_xaxis offset:0 atIndex:13];
            [enc setBuffer:b_spos offset:0 atIndex:14];
            [enc setBuffer:b_scom offset:0 atIndex:15];
            [enc setBuffer:b_dM offset:0 atIndex:16];
            [enc setBuffer:b_cvec offset:0 atIndex:17];
            [enc setBuffer:b_qacc_in offset:0 atIndex:18];
            [enc setBuffer:b_ihold offset:0 atIndex:19];
            [enc setBuffer:b_sidx offset:0 atIndex:20];
            [enc setBuffer:b_melb offset:0 atIndex:21];
            [enc setBuffer:b_meta offset:0 atIndex:22];
            [enc setBuffer:b_csrout offset:0 atIndex:23];
            [enc setBuffer:b_ctrl_out offset:0 atIndex:24];
            [enc setBuffer:b_st_out offset:0 atIndex:25];
            [enc setBuffer:b_scr_out offset:0 atIndex:26];
            const NSUInteger tg = std::min<NSUInteger>(32u, n_st);
            [enc dispatchThreads:MTLSizeMake(n_st, 1, 1)
                threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
            [enc endEncoding];

            if (do_dense) {
                id<MTLComputeCommandEncoder> denc = [cmd computeCommandEncoder];
                [denc setComputePipelineState:pso_dense];
                [denc setBuffer:b_params offset:0 atIndex:0];
                [denc setBytes:&dsc length:sizeof(dsc) atIndex:1];
                [denc setBuffer:b_cmds offset:0 atIndex:2];
                [denc setBuffer:b_qpos offset:0 atIndex:3];
                [denc setBuffer:b_qvel offset:0 atIndex:4];
                [denc setBuffer:b_bias offset:0 atIndex:5];
                [denc setBuffer:b_xmat offset:0 atIndex:6];
                [denc setBuffer:b_xpos offset:0 atIndex:7];
                [denc setBuffer:b_xanchor offset:0 atIndex:8];
                [denc setBuffer:b_xaxis offset:0 atIndex:9];
                [denc setBuffer:b_spos offset:0 atIndex:10];
                [denc setBuffer:b_scom offset:0 atIndex:11];
                [denc setBuffer:b_qM_dense offset:0 atIndex:12];
                [denc setBuffer:b_cvec offset:0 atIndex:13];
                [denc setBuffer:b_qacc_in offset:0 atIndex:14];
                [denc setBuffer:b_ihold offset:0 atIndex:15];
                [denc setBuffer:b_sidx offset:0 atIndex:16];
                [denc setBuffer:b_dense_out offset:0 atIndex:17];
                [denc setBuffer:b_ctrl_dense offset:0 atIndex:18];
                [denc dispatchThreads:MTLSizeMake(n_st, 1, 1)
                    threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
                [denc endEncoding];
            }
        }

        [cmd commit];
        [cmd waitUntilCompleted];
        if ([cmd status] != MTLCommandBufferStatusCompleted) {
            NSError *cmdErr = [cmd error];
            out->error_message = std::string("Metal command failed: ") +
                                 (cmdErr != nil ? [[cmdErr localizedDescription] UTF8String] : "unknown");
            return out;
        }

        out->leg_kin_out = read_vec_buffer<LegKinematicsResult<float>>(b_leg_out, n_leg);
        const auto metas = read_vec_buffer<Sp4StateResultMetaHost>(b_meta, n_st);
        const auto csrouts = read_vec_buffer<float>(b_csrout, n_st * nv);
        const auto denseouts = do_dense ? read_vec_buffer<float>(b_dense_out, n_st * nv) : std::vector<float>{};
        const auto ctrls_dense = do_dense ? read_vec_buffer<float>(b_ctrl_dense, n_st * nu) : std::vector<float>{};
        const auto ctrls_all = read_vec_buffer<float>(b_ctrl_out, 5u * n_st * nu);
        const auto st_outs = read_vec_buffer<SpindleM2K2State<float>>(b_st_out, n_st);
        const auto scr_outs = read_vec_buffer<SpindleM2K2StepScratch<float>>(b_scr_out, n_st);

        out->state_out.resize(n_st);
        for (std::size_t w = 0; w < n_st; ++w) {
            auto &dst = out->state_out[w];
            dst.has_min_lean = metas[w].has_min_lean;
            dst.min_lean_rad = metas[w].min_lean_rad;
            dst.upper_targets = {
                metas[w].upper_targets[0],
                metas[w].upper_targets[1],
                metas[w].upper_targets[2],
                metas[w].upper_targets[3],
                metas[w].upper_targets[4],
            };
            dst.torso_sat = metas[w].torso_sat;
            dst.arms_sat = metas[w].arms_sat;
            dst.f_weight[0] = metas[w].f_weight[0];
            dst.f_weight[1] = metas[w].f_weight[1];
            dst.f_weight_null_subm[0] = metas[w].f_weight_null_subm[0];
            dst.f_weight_null_subm[1] = metas[w].f_weight_null_subm[1];
            dst.csr_out.assign(csrouts.begin() + static_cast<std::ptrdiff_t>(w * nv),
                               csrouts.begin() + static_cast<std::ptrdiff_t>((w + 1) * nv));
            if (do_dense) {
                dst.dense_out.assign(denseouts.begin() + static_cast<std::ptrdiff_t>(w * nv),
                                     denseouts.begin() + static_cast<std::ptrdiff_t>((w + 1) * nv));
                dst.ctrl_dense_pass3.assign(ctrls_dense.begin() + static_cast<std::ptrdiff_t>(w * nu),
                                            ctrls_dense.begin() + static_cast<std::ptrdiff_t>((w + 1) * nu));
            }
            const auto ctrl_slice = [&](std::size_t slot) {
                const std::ptrdiff_t off = static_cast<std::ptrdiff_t>((5u * w + slot) * nu);
                return std::pair{
                    ctrls_all.begin() + off,
                    ctrls_all.begin() + off + static_cast<std::ptrdiff_t>(nu),
                };
            };
            {
                const auto [b, e] = ctrl_slice(0u);
                dst.ctrl_pass0.assign(b, e);
            }
            {
                const auto [b, e] = ctrl_slice(1u);
                dst.ctrl_pass1.assign(b, e);
            }
            {
                const auto [b, e] = ctrl_slice(2u);
                dst.ctrl_pass2.assign(b, e);
            }
            {
                const auto [b, e] = ctrl_slice(3u);
                dst.ctrl_pass3.assign(b, e);
            }
            {
                const auto [b, e] = ctrl_slice(4u);
                dst.ctrl_held.assign(b, e);
            }
            dst.final_state = st_outs[w];
            dst.final_scratch = scr_outs[w];
        }
    }
    return out;
}

} // namespace bike::physics_core::test_sp4_metal
