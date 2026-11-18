#include "test_sp3_drivetrain_metal_helper.hpp"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <span>
#include <sstream>
#include <string>
#include <vector>

namespace bike::physics_core::test_sp3_metal {

namespace {

// The single raw pointer -> std::span boundary of this Metal test helper:
// [MTLBuffer contents] exposes each MTLResourceStorageModeShared buffer as a
// raw void* over its allocated byte length.
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
        "ratchet.hpp",
        "shifting.hpp",
        "pedaling.hpp",
        "assist.hpp"
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
struct ShiftingStepInputMsl {
    float cadence_rpm;
    float required_cadence_rpm;
    float dt;
    float rear_slip_mps;
    float q_driver;
    float q_driven;
    bike::physics_core::RiderShiftInput rider;
    uchar pedaling;
    uchar braking;
    uchar rear_in_contact;
    uchar has_rear_slip;
};

struct ShiftingStepRecordMsl {
    bike::physics_core::ShiftingState<float> state;
    bike::physics_core::RatchetBoundState<float> ratchet;
    float tendon_ranges[26];
    float torque_factor;
    uchar shifted;
    uchar status;
    uchar pad[2];
};

struct PedalingStepInputMsl {
    float phase_rad;
    float rate_rad_s;
    float required_cadence_rpm;
    float effort_nm;
    float dt;
    float shaft_rad_s;
    float ceiling_nm;
    float torque_factor;
    float ripple;
    float dq_crank_rebase;
    float dq_wheel_rebase;
    uchar braking;
    uchar reposition_request;
    uchar has_shaft_rad_s;
    uchar has_ceiling_nm;
    uchar cut_after_ceiling;
    uchar do_rebase;
    uchar pad[2];
};

struct PedalingStepRecordMsl {
    bike::physics_core::PedalingOutput<float> output;
    bike::physics_core::PedalingState<float> state;
    float human_crank_torque_nm;
    uchar status;
    uchar torque_status;
    uchar pad[2];
};

struct AssistStepInputMsl {
    float human_nm;
    float cadence_rpm;
    float speed_mps;
    float dt;
    float torque_request_nm;
    float shaft_rpm;
    uchar braking;
    uchar has_torque_request;
    uchar has_shaft_rpm;
    uchar pad;
};

struct AssistStepRecordMsl {
    float torque;
    bike::physics_core::AssistState<float> state;
    uchar status;
    uchar pad[3];
};

kernel void sp3_shifting_trajectory_kernel(
    constant bike::physics_core::ShiftingParams<float> &params [[buffer(0)]],
    constant bike::physics_core::RatchetBoundState<float> &init_ratchet [[buffer(1)]],
    device const ShiftingStepInputMsl *inputs [[buffer(2)]],
    device ShiftingStepRecordMsl *records [[buffer(3)]],
    constant uint &n_steps [[buffer(4)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    ShiftingState<float> st{};
    shifting_reset<float>(&params, &st);
    RatchetBoundState<float> rat = init_ratchet;
    float tendon_ranges[26]{};
    for (int i = 0; i < 12; ++i) {
        tendon_ranges[2 * i + 0] = -1e12f;
        tendon_ranges[2 * i + 1] = (i == st.active_cog_idx && rat.has_boundary != 0)
            ? rat.boundary
            : 1e12f;
    }
    tendon_ranges[24] = -1e12f;
    tendon_ranges[25] = 0.0f;

    for (uint i = 0u; i < n_steps; ++i) {
        const ShiftingStepInputMsl in = inputs[i];
        bool shifted = false;
        ErrCode err = ErrCode::none;
        const Status status = shifting_step_with_cassette<float, GpuToolkit>(
            &params, &st, &rat,
            in.cadence_rpm, in.required_cadence_rpm, in.dt,
            in.pedaling != 0, in.braking != 0, in.rear_in_contact != 0,
            in.has_rear_slip != 0, in.rear_slip_mps, in.rider,
            in.q_driver, in.q_driven, 13, tendon_ranges, shifted, err);

        ShiftingStepRecordMsl rec{};
        rec.state = st;
        rec.ratchet = rat;
        for (int k = 0; k < 26; ++k) {
            rec.tendon_ranges[k] = tendon_ranges[k];
        }
        rec.torque_factor = shift_torque_factor<float>(&params, &st);
        rec.shifted = shifted ? uchar(1) : uchar(0);
        rec.status = static_cast<uchar>(status);
        records[i] = rec;
    }
}

kernel void sp3_pedaling_trajectory_kernel(
    constant bike::physics_core::PedalingParams<float> &params [[buffer(0)]],
    device const PedalingStepInputMsl *inputs [[buffer(1)]],
    device PedalingStepRecordMsl *records [[buffer(2)]],
    constant uint &n_steps [[buffer(3)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    PedalingState<float> st{};
    pedaling_reset<float>(&st);

    for (uint i = 0u; i < n_steps; ++i) {
        const PedalingStepInputMsl in = inputs[i];
        if (in.do_rebase != 0) {
            rebase_pedaling_2pi<float>(st, in.dq_crank_rebase, in.dq_wheel_rebase);
        }
        PedalingOutput<float> out{};
        ErrCode err = ErrCode::none;
        const Status status = pedaling_update<float, GpuToolkit>(
            &params, &st,
            in.phase_rad, in.rate_rad_s, in.required_cadence_rpm,
            in.effort_nm, in.dt,
            in.braking != 0, in.reposition_request != 0,
            in.has_shaft_rad_s != 0, in.shaft_rad_s,
            in.has_ceiling_nm != 0, in.ceiling_nm,
            in.torque_factor, in.cut_after_ceiling != 0,
            out, err);

        float h_tq = 0.0f;
        ErrCode t_err = ErrCode::none;
        const Status t_status = human_crank_torque<float, GpuToolkit>(
            out.effort_nm, in.phase_rad, in.ripple, h_tq, t_err);

        PedalingStepRecordMsl rec{};
        rec.output = out;
        rec.state = st;
        rec.human_crank_torque_nm = h_tq;
        rec.status = static_cast<uchar>(status);
        rec.torque_status = static_cast<uchar>(t_status);
        records[i] = rec;
    }
}

kernel void sp3_assist_trajectory_kernel(
    constant bike::physics_core::AssistParams<float> &params [[buffer(0)]],
    device const AssistStepInputMsl *inputs [[buffer(1)]],
    device AssistStepRecordMsl *records [[buffer(2)]],
    constant uint &n_steps [[buffer(3)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    AssistState<float> st{};
    assist_reset<float>(&st);

    for (uint i = 0u; i < n_steps; ++i) {
        const AssistStepInputMsl in = inputs[i];
        float tq = 0.0f;
        ErrCode err = ErrCode::none;
        const Status status = assist_step<float, GpuToolkit>(
            &params, &st,
            in.human_nm, in.cadence_rpm, in.speed_mps,
            in.braking != 0, in.dt,
            in.has_torque_request != 0, in.torque_request_nm,
            in.has_shaft_rpm != 0, in.shaft_rpm,
            tq, err);

        AssistStepRecordMsl rec{};
        rec.torque = tq;
        rec.state = st;
        rec.status = static_cast<uchar>(status);
        records[i] = rec;
    }
}

// Address-space torture check (MSL 3.0 & 3.1): compile constant, device, and thread
// Params pointers across all SP3-A entry points to prove zero member functions / address-space friction.
kernel void sp3_torture_address_spaces_kernel(
    constant bike::physics_core::ShiftingParams<float> &c_shift [[buffer(0)]],
    device const bike::physics_core::ShiftingParams<float> *d_shift [[buffer(1)]],
    constant bike::physics_core::PedalingParams<float> &c_pedal [[buffer(2)]],
    device const bike::physics_core::PedalingParams<float> *d_pedal [[buffer(3)]],
    constant bike::physics_core::AssistParams<float> &c_assist [[buffer(4)]],
    device const bike::physics_core::AssistParams<float> *d_assist [[buffer(5)]],
    device float *out_vals [[buffer(6)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    ErrCode err = ErrCode::none;
    bool shifted = false;

    ShiftingParams<float> t_shift = *d_shift;
    ShiftingState<float> s_c{};
    ShiftingState<float> s_d{};
    ShiftingState<float> s_t{};
    shifting_reset<float>(&c_shift, &s_c);
    shifting_reset<float>(d_shift, &s_d);
    shifting_reset<float>(&t_shift, &s_t);
    (void)shifting_update<float, GpuToolkit>(&c_shift, &s_c, 115.0f, 115.0f, 0.00125f, true, false, true, false, 0.0f, RiderShiftInput{}, shifted, err);
    (void)shifting_update<float, GpuToolkit>(d_shift, &s_d, 115.0f, 115.0f, 0.00125f, true, false, true, false, 0.0f, RiderShiftInput{}, shifted, err);
    (void)shifting_update<float, GpuToolkit>(&t_shift, &s_t, 115.0f, 115.0f, 0.00125f, true, false, true, false, 0.0f, RiderShiftInput{}, shifted, err);

    PedalingParams<float> t_pedal = *d_pedal;
    PedalingState<float> p_c{};
    PedalingState<float> p_d{};
    PedalingState<float> p_t{};
    PedalingOutput<float> po{};
    (void)pedaling_update<float, GpuToolkit>(&c_pedal, &p_c, 0.2f, 8.0f, 80.0f, 30.0f, 0.00125f, false, false, false, 0.0f, false, 0.0f, 1.0f, false, po, err);
    (void)pedaling_update<float, GpuToolkit>(d_pedal, &p_d, 0.2f, 8.0f, 80.0f, 30.0f, 0.00125f, false, false, false, 0.0f, false, 0.0f, 1.0f, false, po, err);
    (void)pedaling_update<float, GpuToolkit>(&t_pedal, &p_t, 0.2f, 8.0f, 80.0f, 30.0f, 0.00125f, false, false, false, 0.0f, false, 0.0f, 1.0f, false, po, err);
    rebase_pedaling_2pi<float>(p_t, 6.2831853f, 6.2831853f);

    AssistParams<float> t_assist = *d_assist;
    AssistState<float> a_c{};
    AssistState<float> a_d{};
    AssistState<float> a_t{};
    float tq_c = 0.0f, tq_d = 0.0f, tq_t = 0.0f;
    (void)assist_step<float, GpuToolkit>(&c_assist, &a_c, 25.0f, 80.0f, 4.0f, false, 0.00125f, false, 0.0f, false, 0.0f, tq_c, err);
    (void)assist_step<float, GpuToolkit>(d_assist, &a_d, 25.0f, 80.0f, 4.0f, false, 0.00125f, false, 0.0f, false, 0.0f, tq_d, err);
    (void)assist_step<float, GpuToolkit>(&t_assist, &a_t, 25.0f, 80.0f, 4.0f, false, 0.00125f, false, 0.0f, false, 0.0f, tq_t, err);

    out_vals[0] = shift_torque_factor<float>(&c_shift, &s_c) + po.effort_nm + tq_c + tq_d + tq_t;
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

static_assert(sizeof(ShiftingStepInput) == 36);
static_assert(sizeof(ShiftingStepRecord) == 160);
static_assert(sizeof(PedalingStepInput) == 52);
static_assert(sizeof(PedalingStepRecord) == 80);
static_assert(sizeof(AssistStepInput) == 28);
static_assert(sizeof(AssistStepRecord) == 32);

} // namespace

MetalSp3Output run_metal_sp3_parity(const MetalSp3Request &req) {
    MetalSp3Output out;
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

        // Verify MSL 3.0 & 3.1 compilation (matching warp-metal wp_metal_load_library)
        for (const MTLLanguageVersion ver : {MTLLanguageVersion3_0, MTLLanguageVersion3_1}) {
            MTLCompileOptions *ver_opts = [[MTLCompileOptions alloc] init];
            ver_opts.mathMode = MTLMathModeSafe;
            ver_opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;
            ver_opts.languageVersion = ver;
            NSError *ver_err = nil;
            id<MTLLibrary> ver_lib = [device newLibraryWithSource:ns_src options:ver_opts error:&ver_err];
            if (ver_lib == nil) {
                out.error_message = std::string("Metal MSL 3.x compile error: ") +
                                    (ver_err != nil ? [[ver_err localizedDescription] UTF8String] : "unknown");
                return out;
            }
            id<MTLFunction> torture_fn = [ver_lib newFunctionWithName:@"sp3_torture_address_spaces_kernel"];
            if (torture_fn == nil) {
                out.error_message = "Missing sp3_torture_address_spaces_kernel in MSL 3.x library";
                return out;
            }
            id<MTLComputePipelineState> torture_pso =
                [device newComputePipelineStateWithFunction:torture_fn error:&ver_err];
            if (torture_pso == nil) {
                out.error_message = std::string("Metal MSL 3.x torture PSO error: ") +
                                    (ver_err != nil ? [[ver_err localizedDescription] UTF8String] : "unknown");
                return out;
            }
            if (ver == MTLLanguageVersion3_0) {
                out.torture_3_0_passed = true;
            } else {
                out.torture_3_1_passed = true;
            }
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

        id<MTLComputePipelineState> pso_shift = make_pso(@"sp3_shifting_trajectory_kernel");
        id<MTLComputePipelineState> pso_pedal = make_pso(@"sp3_pedaling_trajectory_kernel");
        id<MTLComputePipelineState> pso_assist = make_pso(@"sp3_assist_trajectory_kernel");
        if (pso_shift == nil || pso_pedal == nil || pso_assist == nil) {
            return out;
        }

        id<MTLCommandQueue> queue = [device newCommandQueue];
        if (queue == nil) {
            out.error_message = "Metal newCommandQueue failed";
            return out;
        }

        const std::size_t n_shift = req.shift_steps.size();
        const std::size_t n_pedal = req.pedal_steps.size();
        const std::size_t n_assist = req.assist_steps.size();

        id<MTLBuffer> b_shift_in = make_vec_buffer(device, req.shift_steps);
        id<MTLBuffer> b_shift_out = make_empty_buffer<ShiftingStepRecord>(device, n_shift);

        id<MTLBuffer> b_pedal_in = make_vec_buffer(device, req.pedal_steps);
        id<MTLBuffer> b_pedal_out = make_empty_buffer<PedalingStepRecord>(device, n_pedal);

        id<MTLBuffer> b_assist_in = make_vec_buffer(device, req.assist_steps);
        id<MTLBuffer> b_assist_out = make_empty_buffer<AssistStepRecord>(device, n_assist);

        id<MTLCommandBuffer> cmd = [queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];

        if (n_shift > 0) {
            [enc setComputePipelineState:pso_shift];
            [enc setBytes:&req.shift_params length:sizeof(req.shift_params) atIndex:0];
            [enc setBytes:&req.shift_init_ratchet length:sizeof(req.shift_init_ratchet) atIndex:1];
            [enc setBuffer:b_shift_in offset:0 atIndex:2];
            [enc setBuffer:b_shift_out offset:0 atIndex:3];
            const std::uint32_t ns = static_cast<std::uint32_t>(n_shift);
            [enc setBytes:&ns length:sizeof(ns) atIndex:4];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
        }

        if (n_pedal > 0) {
            [enc setComputePipelineState:pso_pedal];
            [enc setBytes:&req.pedal_params length:sizeof(req.pedal_params) atIndex:0];
            [enc setBuffer:b_pedal_in offset:0 atIndex:1];
            [enc setBuffer:b_pedal_out offset:0 atIndex:2];
            const std::uint32_t np = static_cast<std::uint32_t>(n_pedal);
            [enc setBytes:&np length:sizeof(np) atIndex:3];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
           threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
        }

        if (n_assist > 0) {
            [enc setComputePipelineState:pso_assist];
            [enc setBytes:&req.assist_params length:sizeof(req.assist_params) atIndex:0];
            [enc setBuffer:b_assist_in offset:0 atIndex:1];
            [enc setBuffer:b_assist_out offset:0 atIndex:2];
            const std::uint32_t na = static_cast<std::uint32_t>(n_assist);
            [enc setBytes:&na length:sizeof(na) atIndex:3];
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

        out.shift_records = read_vec_buffer<ShiftingStepRecord>(b_shift_out, n_shift);
        out.pedal_records = read_vec_buffer<PedalingStepRecord>(b_pedal_out, n_pedal);
        out.assist_records = read_vec_buffer<AssistStepRecord>(b_assist_out, n_assist);
    }
    return out;
}

} // namespace bike::physics_core::test_sp3_metal
