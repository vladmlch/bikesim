#include "test_sp2_force_laws_metal_helper.hpp"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace bike::physics_core::test_sp2_metal {

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
    const std::array<const char *, 10> headers = {
        "common.hpp",
        "gpu_math.hpp",
        "gpu_toolkit.hpp",
        "tyre_types.hpp",
        "kinematics.hpp",
        "suspension.hpp",
        "resistance.hpp",
        "brake_hold.hpp",
        "joint_envelope.hpp",
        "ratchet.hpp"
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
kernel void sp2_math_kernel(
    device const float *log_in [[buffer(0)]],
    device const float *pow_base [[buffer(1)]],
    device const float *pow_exp [[buffer(2)]],
    device float *log_out [[buffer(3)]],
    device float *pow_out [[buffer(4)]],
    constant uint &n_log [[buffer(5)]],
    constant uint &n_pow [[buffer(6)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid < n_log) {
        log_out[gid] = GpuToolkit::log(log_in[gid]);
    }
    if (gid < n_pow) {
        pow_out[gid] = GpuToolkit::pow_py(pow_base[gid], pow_exp[gid]);
    }
}

kernel void sp2_suspension_kernel(
    device const bike::physics_core::SuspensionParams<float> *params [[buffer(0)]],
    device const float *fork_q [[buffer(1)]],
    device const float *fork_v [[buffer(2)]],
    device const float *shock_q [[buffer(3)]],
    device const float *shock_v [[buffer(4)]],
    device bike::physics_core::SuspensionResult<float> *out [[buffer(5)]],
    device uchar *out_status [[buffer(6)]],
    constant uint &n [[buffer(7)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid >= n) {
        return;
    }
    SuspensionResult<float> local_res{};
    ErrCode err = ErrCode::none;
    const Status st = eval_suspension<float, GpuToolkit>(
        &params[gid], fork_q[gid], fork_v[gid], shock_q[gid], shock_v[gid],
        local_res, err);
    out[gid] = local_res;
    out_status[gid] = static_cast<uchar>(st);
}

struct ResistanceArgs {
    bike::physics_core::ResistanceParams<float> cfg;
    bike::physics_core::GpuWheelChain wheel_chains[2];
    bike::physics_core::GpuWheelChain frame_chain;
    int nv;
    int njnt;
    int front_n;
    int rear_n;
    float front_eff_r;
    float rear_eff_r;
};

kernel void sp2_resistance_kernel(
    constant ResistanceArgs &args [[buffer(0)]],
    device const float *body_xpos [[buffer(1)]],
    device const float *jnt_xanchor [[buffer(2)]],
    device const float *jnt_xaxis [[buffer(3)]],
    device const float *qvel [[buffer(4)]],
    device const float *front_loads [[buffer(5)]],
    device const uchar *front_work [[buffer(6)]],
    device const float *front_xpos [[buffer(7)]],
    device const float *front_xmat [[buffer(8)]],
    device const float *rear_loads [[buffer(9)]],
    device const uchar *rear_work [[buffer(10)]],
    device const float *rear_xpos [[buffer(11)]],
    device const float *rear_xmat [[buffer(12)]],
    device const float *frame_xpos [[buffer(13)]],
    device const float *frame_xmat [[buffer(14)]],
    device float *rolling_out [[buffer(15)]],
    device float *aero_out [[buffer(16)]],
    device uchar *out_status [[buffer(17)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    float jr_scratch[3 * kMaxNv]{};
    float jp_scratch[3 * kMaxNv]{};
    float row_scratch[kMaxNv]{};

    ChainWheelJrFn<float, GpuToolkit, device const float *, constant GpuWheelChain *> wheel_jr{
        args.wheel_chains, jnt_xaxis, args.nv, args.njnt};
    ChainFrameJpFn<float, GpuToolkit, device const float *, device const float *, device const float *> frame_jp{
        args.frame_chain, body_xpos, jnt_xanchor, jnt_xaxis, args.nv, args.njnt};

    ErrCode err = ErrCode::none;
    ResDiagField diag = ResDiagField::none;
    const Status st = eval_resistance<float, GpuToolkit>(
        args.nv, &args.cfg, qvel,
        front_loads, front_work, args.front_n, args.front_eff_r, front_xpos, front_xmat,
        rear_loads, rear_work, args.rear_n, args.rear_eff_r, rear_xpos, rear_xmat,
        frame_xpos, frame_xmat,
        jr_scratch, jp_scratch, row_scratch,
        rolling_out, aero_out,
        wheel_jr, frame_jp, err, diag);
    *out_status = static_cast<uchar>(st);
}

struct BrakeInArgs {
    bike::physics_core::BrakeHoldParams<float> spec;
    float ceiling_nm;
    float front_demand;
    float rear_demand;
    float theta_init;
    float omega;
    float latch_init;
};

struct BrakeOutArgs {
    float loss_front;
    float loss_rear;
    float hold_torque;
    float rebased_theta;
    float rebased_latch;
    uchar changed;
    uchar has_latch;
};

kernel void sp2_brake_kernel(
    constant BrakeInArgs &in_args [[buffer(0)]],
    device BrakeOutArgs *out [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    float frictionloss[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool changed = false;
    ErrCode err = ErrCode::none;
    (void)apply_static_brake<float, GpuToolkit>(
        in_args.ceiling_nm, in_args.front_demand, in_args.rear_demand,
        1, 3, 4, frictionloss, changed, err);

    float theta = in_args.theta_init;
    float latch = in_args.latch_init;
    bool has_latch = true;
    (void)rebase_2pi_angle<float, GpuToolkit>(theta, has_latch, latch);
    float torque = 0.0f;
    (void)eval_brake_hold_wheel<float, GpuToolkit>(
        &in_args.spec, in_args.front_demand, theta, in_args.omega,
        true, false, has_latch, latch, torque, err);

    out->loss_front = frictionloss[1];
    out->loss_rear = frictionloss[3];
    out->changed = changed ? 1 : 0;
    out->hold_torque = torque;
    out->rebased_theta = theta;
    out->rebased_latch = latch;
    out->has_latch = has_latch ? 1 : 0;
}

kernel void sp2_envelope_kernel(
    constant bike::physics_core::RiderJointEnvelopeSpec<float> &spec [[buffer(0)]],
    device const float *qpos [[buffer(1)]],
    device float *qfrc_out [[buffer(2)]],
    device float *stored_out [[buffer(3)]],
    device uchar *status_out [[buffer(4)]],
    constant int &nq [[buffer(5)]],
    constant int &nv [[buffer(6)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    for (int i = 0; i < nv; ++i) {
        qfrc_out[i] = 0.0f;
    }
    float stored = 0.0f;
    ErrCode err = ErrCode::none;
    const Status st = eval_rider_joint_envelope<float, GpuToolkit>(
        &spec, nq, nv, qpos, qfrc_out, stored, err);
    *stored_out = stored;
    *status_out = static_cast<uchar>(st);
}

struct RatchetInArgs {
    bike::physics_core::WorldRotationsRebaseState<float> rebase_in;
    float old_ratio;
    float new_ratio;
    int active_cog;
};

struct RatchetOutArgs {
    bike::physics_core::WorldRotationsRebaseState<float> rebase_out;
    float phi;
    float bound_after_reset;
    float bound_after_prepare;
    float bound_after_shift;
    float cassette_ranges[24];
    uchar status;
};

kernel void sp2_ratchet_kernel(
    constant RatchetInArgs &in_args [[buffer(0)]],
    device RatchetOutArgs *out [[buffer(1)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    WorldRotationsRebaseState<float> st_reb = in_args.rebase_in;
    ErrCode err = ErrCode::none;
    Status st = rebase_world_rotations_2pi<float, GpuToolkit>(&st_reb, err);
    out->rebase_out = st_reb;

    bool has_b = false;
    float b = 0.0f;
    const float q_drv = st_reb.drive_shaft_qpos;
    const float q_rwn = st_reb.rear_wheel_qpos;
    out->phi = ratchet_phi<float, GpuToolkit>(in_args.old_ratio, q_drv, q_rwn);
    if (st == Status::ok) {
        st = ratchet_reset_bound<float, GpuToolkit>(
            in_args.old_ratio, q_drv, q_rwn, has_b, b, err);
    }
    out->bound_after_reset = b;
    if (st == Status::ok) {
        st = ratchet_prepare_bound<float, GpuToolkit>(
            in_args.old_ratio, q_drv - 0.001f, q_rwn, has_b, b, err);
    }
    out->bound_after_prepare = b;
    if (st == Status::ok) {
        st = ratchet_shift_ratio_bound<float, GpuToolkit>(
            in_args.old_ratio, in_args.new_ratio, q_drv - 0.001f, q_rwn,
            has_b, b, err);
    }
    out->bound_after_shift = b;

    i32 tids[kCassetteCogCount] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    float ranges[24]{};
    if (st == Status::ok) {
        st = fill_cassette_tendon_ranges<float, GpuToolkit>(
            in_args.active_cog, kCassetteCogCount, kCassetteCogCount, b,
            tids, ranges, err);
    }
    for (int i = 0; i < 24; ++i) {
        out->cassette_ranges[i] = ranges[i];
    }
    out->status = static_cast<uchar>(st);
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
id<MTLBuffer> make_empty_buffer(id<MTLDevice> device, std::size_t count) {
    const std::size_t bytes = count * sizeof(T);
    return [device newBufferWithLength:(bytes > 0 ? bytes : 4u)
                               options:MTLResourceStorageModeShared];
}

template <typename T>
std::vector<T> read_vec_buffer(id<MTLBuffer> buf, std::size_t count) {
    std::vector<T> out(count);
    const std::span<const T> src = boundary_span(
        static_cast<const T *>([buf contents]), count);
    std::ranges::copy(src, out.begin());
    return out;
}

struct ResistanceArgsHost {
    ResistanceParams<float> cfg;
    GpuWheelChain wheel_chains[2];
    GpuWheelChain frame_chain;
    i32 nv;
    i32 njnt;
    i32 front_n;
    i32 rear_n;
    float front_eff_r;
    float rear_eff_r;
};

struct BrakeInArgsHost {
    BrakeHoldParams<float> spec;
    float ceiling_nm;
    float front_demand;
    float rear_demand;
    float theta_init;
    float omega;
    float latch_init;
};

struct BrakeOutArgsHost {
    float loss_front;
    float loss_rear;
    float hold_torque;
    float rebased_theta;
    float rebased_latch;
    u8 changed;
    u8 has_latch;
};

struct RatchetInArgsHost {
    WorldRotationsRebaseState<float> rebase_in;
    float old_ratio;
    float new_ratio;
    i32 active_cog;
};

struct RatchetOutArgsHost {
    WorldRotationsRebaseState<float> rebase_out;
    float phi;
    float bound_after_reset;
    float bound_after_prepare;
    float bound_after_shift;
    float cassette_ranges[24];
    u8 status;
};

} // namespace

MetalSp2Output run_metal_sp2_parity(const MetalSp2Request &req) {
    MetalSp2Output out;
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) {
            out.device_available = false;
            out.error_message = "MTLCreateSystemDefaultDevice() returned nil";
            return out;
        }
        out.device_available = true;
        out.device_name = [[device name] UTF8String];

        std::string src_err;
        const std::string msl_src = build_msl_source(req.physics_core_dir, src_err);
        if (msl_src.empty()) {
            out.error_message = src_err;
            return out;
        }

        MTLCompileOptions *opts = [[MTLCompileOptions alloc] init];
        opts.mathMode = MTLMathModeSafe;
        opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;

        NSError *err = nil;
        NSString *ns_src = [NSString stringWithUTF8String:msl_src.c_str()];
        id<MTLLibrary> library = [device newLibraryWithSource:ns_src options:opts error:&err];
        if (library == nil) {
            out.error_message = std::string("Metal MSL compile error: ") +
                                (err != nil ? [[err localizedDescription] UTF8String] : "unknown");
            return out;
        }

        auto make_pso = [&](NSString *name) -> id<MTLComputePipelineState> {
            id<MTLFunction> fn = [library newFunctionWithName:name];
            if (fn == nil) {
                out.error_message = std::string("Missing Metal function: ") + [name UTF8String];
                return nil;
            }
            id<MTLComputePipelineState> pso =
                [device newComputePipelineStateWithFunction:fn error:&err];
            if (pso == nil) {
                out.error_message = std::string("Metal PSO error: ") +
                                    (err != nil ? [[err localizedDescription] UTF8String] : "unknown");
                return nil;
            }
            return pso;
        };

        id<MTLComputePipelineState> pso_math = make_pso(@"sp2_math_kernel");
        id<MTLComputePipelineState> pso_susp = make_pso(@"sp2_suspension_kernel");
        id<MTLComputePipelineState> pso_res = make_pso(@"sp2_resistance_kernel");
        id<MTLComputePipelineState> pso_brake = make_pso(@"sp2_brake_kernel");
        id<MTLComputePipelineState> pso_env = make_pso(@"sp2_envelope_kernel");
        id<MTLComputePipelineState> pso_ratchet = make_pso(@"sp2_ratchet_kernel");
        if (pso_math == nil || pso_susp == nil || pso_res == nil ||
            pso_brake == nil || pso_env == nil || pso_ratchet == nil) {
            return out;
        }

        id<MTLCommandQueue> queue = [device newCommandQueue];
        if (queue == nil) {
            out.error_message = "Metal newCommandQueue failed";
            return out;
        }

        // 1. Math buffers
        const std::size_t n_log = req.log_in.size();
        const std::size_t n_pow = req.pow_base.size();
        id<MTLBuffer> b_log_in = make_vec_buffer(device, req.log_in);
        id<MTLBuffer> b_pow_b = make_vec_buffer(device, req.pow_base);
        id<MTLBuffer> b_pow_e = make_vec_buffer(device, req.pow_exp);
        id<MTLBuffer> b_log_out = make_empty_buffer<float>(device, n_log);
        id<MTLBuffer> b_pow_out = make_empty_buffer<float>(device, n_pow);

        // 2. Suspension buffers
        const std::size_t n_susp = req.susp_params.size();
        id<MTLBuffer> b_susp_p = make_vec_buffer(device, req.susp_params);
        id<MTLBuffer> b_susp_fq = make_vec_buffer(device, req.susp_fork_q);
        id<MTLBuffer> b_susp_fv = make_vec_buffer(device, req.susp_fork_v);
        id<MTLBuffer> b_susp_sq = make_vec_buffer(device, req.susp_shock_q);
        id<MTLBuffer> b_susp_sv = make_vec_buffer(device, req.susp_shock_v);
        id<MTLBuffer> b_susp_out = make_empty_buffer<SuspensionResult<float>>(device, n_susp);
        id<MTLBuffer> b_susp_st = make_empty_buffer<u8>(device, n_susp);

        // 3. Resistance buffers
        ResistanceArgsHost r_args{
            .cfg = req.res_cfg,
            .wheel_chains = {req.res_wheel_chains[0], req.res_wheel_chains[1]},
            .frame_chain = req.res_frame_chain,
            .nv = req.res_nv,
            .njnt = req.res_njnt,
            .front_n = static_cast<i32>(req.res_front_loads.size()),
            .rear_n = static_cast<i32>(req.res_rear_loads.size()),
            .front_eff_r = req.res_front_eff_r,
            .rear_eff_r = req.res_rear_eff_r,
        };
        id<MTLBuffer> b_body_xpos = make_vec_buffer(device, req.res_body_xpos);
        id<MTLBuffer> b_jnt_xa = make_vec_buffer(device, req.res_jnt_xanchor);
        id<MTLBuffer> b_jnt_xx = make_vec_buffer(device, req.res_jnt_xaxis);
        id<MTLBuffer> b_qvel = make_vec_buffer(device, req.res_qvel);
        id<MTLBuffer> b_fl = make_vec_buffer(device, req.res_front_loads);
        id<MTLBuffer> b_fw = make_vec_buffer(device, req.res_front_work);
        id<MTLBuffer> b_fxp = make_vec_buffer(device, req.res_front_xpos);
        id<MTLBuffer> b_fxm = make_vec_buffer(device, req.res_front_xmat);
        id<MTLBuffer> b_rl = make_vec_buffer(device, req.res_rear_loads);
        id<MTLBuffer> b_rw = make_vec_buffer(device, req.res_rear_work);
        id<MTLBuffer> b_rxp = make_vec_buffer(device, req.res_rear_xpos);
        id<MTLBuffer> b_rxm = make_vec_buffer(device, req.res_rear_xmat);
        id<MTLBuffer> b_frxp = make_vec_buffer(device, req.res_frame_xpos);
        id<MTLBuffer> b_frxm = make_vec_buffer(device, req.res_frame_xmat);
        id<MTLBuffer> b_roll_out = make_empty_buffer<float>(device, static_cast<std::size_t>(req.res_nv));
        id<MTLBuffer> b_aero_out = make_empty_buffer<float>(device, static_cast<std::size_t>(req.res_nv));
        id<MTLBuffer> b_res_st = make_empty_buffer<u8>(device, 1u);

        // 4. Brake buffers
        BrakeInArgsHost b_in{
            .spec = req.brake_spec,
            .ceiling_nm = req.brake_ceiling_nm,
            .front_demand = req.brake_front_demand,
            .rear_demand = req.brake_rear_demand,
            .theta_init = req.brake_theta_init,
            .omega = req.brake_omega,
            .latch_init = req.brake_latch_init,
        };
        id<MTLBuffer> b_brake_out = make_empty_buffer<BrakeOutArgsHost>(device, 1u);

        // 5. Envelope buffers
        id<MTLBuffer> b_env_qpos = make_vec_buffer(device, req.env_qpos);
        id<MTLBuffer> b_env_qfrc = make_empty_buffer<float>(device, static_cast<std::size_t>(req.env_nv));
        id<MTLBuffer> b_env_stored = make_empty_buffer<float>(device, 1u);
        id<MTLBuffer> b_env_st = make_empty_buffer<u8>(device, 1u);

        // 6. Ratchet buffers
        RatchetInArgsHost rat_in{
            .rebase_in = req.rebase_in,
            .old_ratio = req.ratchet_old_ratio,
            .new_ratio = req.ratchet_new_ratio,
            .active_cog = req.active_cog,
        };
        id<MTLBuffer> b_rat_out = make_empty_buffer<RatchetOutArgsHost>(device, 1u);

        id<MTLCommandBuffer> cmd = [queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];

        // Dispatch 1: math
        {
            [enc setComputePipelineState:pso_math];
            [enc setBuffer:b_log_in offset:0 atIndex:0];
            [enc setBuffer:b_pow_b offset:0 atIndex:1];
            [enc setBuffer:b_pow_e offset:0 atIndex:2];
            [enc setBuffer:b_log_out offset:0 atIndex:3];
            [enc setBuffer:b_pow_out offset:0 atIndex:4];
            const std::uint32_t nl = static_cast<std::uint32_t>(n_log);
            const std::uint32_t np = static_cast<std::uint32_t>(n_pow);
            [enc setBytes:&nl length:sizeof(nl) atIndex:5];
            [enc setBytes:&np length:sizeof(np) atIndex:6];
            const NSUInteger nmax = std::max(n_log, n_pow);
            const NSUInteger tg = 64;
            const NSUInteger grid = ((nmax + tg - 1u) / tg) * tg;
            [enc dispatchThreads:MTLSizeMake(grid > 0 ? grid : tg, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        }

        // Dispatch 2: suspension
        if (n_susp > 0) {
            [enc setComputePipelineState:pso_susp];
            [enc setBuffer:b_susp_p offset:0 atIndex:0];
            [enc setBuffer:b_susp_fq offset:0 atIndex:1];
            [enc setBuffer:b_susp_fv offset:0 atIndex:2];
            [enc setBuffer:b_susp_sq offset:0 atIndex:3];
            [enc setBuffer:b_susp_sv offset:0 atIndex:4];
            [enc setBuffer:b_susp_out offset:0 atIndex:5];
            [enc setBuffer:b_susp_st offset:0 atIndex:6];
            const std::uint32_t ns = static_cast<std::uint32_t>(n_susp);
            [enc setBytes:&ns length:sizeof(ns) atIndex:7];
            const NSUInteger tg = 64;
            const NSUInteger grid = ((n_susp + tg - 1u) / tg) * tg;
            [enc dispatchThreads:MTLSizeMake(grid, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        }

        // Dispatch 3: resistance
        {
            [enc setComputePipelineState:pso_res];
            [enc setBytes:&r_args length:sizeof(r_args) atIndex:0];
            [enc setBuffer:b_body_xpos offset:0 atIndex:1];
            [enc setBuffer:b_jnt_xa offset:0 atIndex:2];
            [enc setBuffer:b_jnt_xx offset:0 atIndex:3];
            [enc setBuffer:b_qvel offset:0 atIndex:4];
            [enc setBuffer:b_fl offset:0 atIndex:5];
            [enc setBuffer:b_fw offset:0 atIndex:6];
            [enc setBuffer:b_fxp offset:0 atIndex:7];
            [enc setBuffer:b_fxm offset:0 atIndex:8];
            [enc setBuffer:b_rl offset:0 atIndex:9];
            [enc setBuffer:b_rw offset:0 atIndex:10];
            [enc setBuffer:b_rxp offset:0 atIndex:11];
            [enc setBuffer:b_rxm offset:0 atIndex:12];
            [enc setBuffer:b_frxp offset:0 atIndex:13];
            [enc setBuffer:b_frxm offset:0 atIndex:14];
            [enc setBuffer:b_roll_out offset:0 atIndex:15];
            [enc setBuffer:b_aero_out offset:0 atIndex:16];
            [enc setBuffer:b_res_st offset:0 atIndex:17];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
        }

        // Dispatch 4: brake
        {
            [enc setComputePipelineState:pso_brake];
            [enc setBytes:&b_in length:sizeof(b_in) atIndex:0];
            [enc setBuffer:b_brake_out offset:0 atIndex:1];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
        }

        // Dispatch 5: envelope
        {
            [enc setComputePipelineState:pso_env];
            [enc setBytes:&req.env_spec length:sizeof(req.env_spec) atIndex:0];
            [enc setBuffer:b_env_qpos offset:0 atIndex:1];
            [enc setBuffer:b_env_qfrc offset:0 atIndex:2];
            [enc setBuffer:b_env_stored offset:0 atIndex:3];
            [enc setBuffer:b_env_st offset:0 atIndex:4];
            [enc setBytes:&req.env_nq length:sizeof(req.env_nq) atIndex:5];
            [enc setBytes:&req.env_nv length:sizeof(req.env_nv) atIndex:6];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
        }

        // Dispatch 6: ratchet
        {
            [enc setComputePipelineState:pso_ratchet];
            [enc setBytes:&rat_in length:sizeof(rat_in) atIndex:0];
            [enc setBuffer:b_rat_out offset:0 atIndex:1];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
        }

        [enc endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];

        if ([cmd status] != MTLCommandBufferStatusCompleted) {
            NSError *cmd_err = [cmd error];
            out.error_message = std::string("Metal command buffer failed: ") +
                                (cmd_err != nil ? [[cmd_err localizedDescription] UTF8String] : "unknown");
            return out;
        }

        out.log_out = read_vec_buffer<float>(b_log_out, n_log);
        out.pow_out = read_vec_buffer<float>(b_pow_out, n_pow);
        out.susp_out = read_vec_buffer<SuspensionResult<float>>(b_susp_out, n_susp);
        out.susp_status = read_vec_buffer<u8>(b_susp_st, n_susp);

        out.res_rolling_out = read_vec_buffer<float>(b_roll_out, static_cast<std::size_t>(req.res_nv));
        out.res_aero_out = read_vec_buffer<float>(b_aero_out, static_cast<std::size_t>(req.res_nv));
        out.res_status = read_vec_buffer<u8>(b_res_st, 1u)[0];

        const BrakeOutArgsHost br = boundary_span(
            static_cast<const BrakeOutArgsHost *>([b_brake_out contents]), 1u)[0];
        out.brake_loss_front = br.loss_front;
        out.brake_loss_rear = br.loss_rear;
        out.brake_changed = br.changed;
        out.brake_hold_torque = br.hold_torque;
        out.brake_rebased_theta = br.rebased_theta;
        out.brake_rebased_latch = br.rebased_latch;
        out.brake_has_latch = br.has_latch;

        out.env_qfrc_out = read_vec_buffer<float>(b_env_qfrc, static_cast<std::size_t>(req.env_nv));
        out.env_stored_j = read_vec_buffer<float>(b_env_stored, 1u)[0];
        out.env_status = read_vec_buffer<u8>(b_env_st, 1u)[0];

        const RatchetOutArgsHost ro = boundary_span(
            static_cast<const RatchetOutArgsHost *>([b_rat_out contents]), 1u)[0];
        out.rebase_out = ro.rebase_out;
        out.ratchet_phi = ro.phi;
        out.ratchet_bound_after_reset = ro.bound_after_reset;
        out.ratchet_bound_after_prepare = ro.bound_after_prepare;
        out.ratchet_bound_after_shift = ro.bound_after_shift;
        out.cassette_ranges.assign(std::begin(ro.cassette_ranges), std::end(ro.cassette_ranges));
        out.ratchet_status = ro.status;
    }
    return out;
}

} // namespace bike::physics_core::test_sp2_metal
