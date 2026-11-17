#include "test_physics_core_metal_helper.hpp"

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

namespace bike::physics_core::test_metal {

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

struct ParityMeta {
    float bike_real_1e_12 = 0.0f;
    std::uint32_t sizeof_tyre_model = 0;
    std::uint32_t sizeof_compliant_state = 0;
    std::uint32_t sizeof_station_state = 0;
};

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
    const std::array<const char *, 4> headers = {
        "common.hpp",
        "gpu_math.hpp",
        "gpu_toolkit.hpp",
        "tyre_types.hpp"
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
struct ParityMeta {
    float bike_real_1e_12;
    uint sizeof_tyre_model;
    uint sizeof_compliant_state;
    uint sizeof_station_state;
};

kernel void physics_core_parity_kernel(
    device const float *exp_in [[buffer(0)]],
    device const float *tanh_in [[buffer(1)]],
    device const float *hypot_a [[buffer(2)]],
    device const float *hypot_b [[buffer(3)]],
    device const float *interp_s [[buffer(4)]],
    device const float *interp_x [[buffer(5)]],
    device const float *interp_x0 [[buffer(6)]],
    device const float *interp_y0 [[buffer(7)]],
    device const float *interp_x1 [[buffer(8)]],
    device const float *interp_y1 [[buffer(9)]],
    device const float *dot3_a [[buffer(10)]],
    device const float *dot3_b [[buffer(11)]],
    device const float *ring_local [[buffer(12)]],
    device const float *ring_rmat [[buffer(13)]],
    device float *exp_out [[buffer(14)]],
    device float *tanh_out [[buffer(15)]],
    device float *hypot_out [[buffer(16)]],
    device float *interp_out [[buffer(17)]],
    device float *dot3_out [[buffer(18)]],
    device float *ring_out [[buffer(19)]],
    device ParityMeta *meta [[buffer(20)]],
    constant uint &n [[buffer(21)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid >= n) {
        return;
    }
    exp_out[gid] = GpuToolkit::exp(exp_in[gid]);
    tanh_out[gid] = GpuToolkit::tanh(tanh_in[gid]);
    hypot_out[gid] = GpuToolkit::hypot2(hypot_a[gid], hypot_b[gid]);
    interp_out[gid] = GpuToolkit::interp_segment(
        interp_s[gid], interp_x[gid],
        interp_x0[gid], interp_y0[gid],
        interp_x1[gid], interp_y1[gid]);
    dot3_out[gid] = GpuToolkit::dot3(&dot3_a[gid * 3u], &dot3_b[gid * 3u]);
    GpuToolkit::ring_lift(1, &ring_local[gid * 3u], ring_rmat, &ring_out[gid * 3u]);
    if (gid == 0u) {
        Result res{};
        res.status = Status::ok;
        res.err = ErrCode::none;
        PolylineView<float> pl{};
        SignedProfileView<float> sp{};
        StationRingView<float> sr{};
        SurfaceMapView<float> sm{};
        (void)res.ok();
        (void)pl.num_vertices;
        (void)sp.num_vertices;
        (void)sr.station_count;
        (void)sm.section_count;
        meta->bike_real_1e_12 = BIKE_REAL(float, 1e-12);
        meta->sizeof_tyre_model = static_cast<uint>(sizeof(TyreModel<float>));
        meta->sizeof_compliant_state = static_cast<uint>(sizeof(CompliantWheelStatePod<float>));
        meta->sizeof_station_state = static_cast<uint>(sizeof(StationStatePod<float>));
    }
}
)MSL";
    return combined;
}

id<MTLBuffer> make_float_buffer(id<MTLDevice> device, const std::vector<float> &vals) {
    const std::size_t bytes = vals.size() * sizeof(float);
    if (bytes == 0) {
        return [device newBufferWithLength:4u options:MTLResourceStorageModeShared];
    }
    return [device newBufferWithBytes:vals.data()
                               length:bytes
                              options:MTLResourceStorageModeShared];
}

id<MTLBuffer> make_empty_float_buffer(id<MTLDevice> device, std::size_t count) {
    const std::size_t bytes = count * sizeof(float);
    return [device newBufferWithLength:(bytes > 0 ? bytes : 4u)
                               options:MTLResourceStorageModeShared];
}

std::vector<float> read_float_buffer(id<MTLBuffer> buf, std::size_t count) {
    std::vector<float> out(count);
    const std::span<const float> src = boundary_span(
        static_cast<const float *>([buf contents]), count);
    std::ranges::copy(src, out.begin());
    return out;
}

} // namespace

MetalParityOutput run_metal_parity_kernel(const MetalParityRequest &req) {
    MetalParityOutput out;
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) {
            out.device_available = false;
            out.error_message = "MTLCreateSystemDefaultDevice() returned nil";
            return out;
        }
        out.device_available = true;
        out.device_name = [[device name] UTF8String];

        const std::size_t n = req.exp_inputs.size();
        if (n == 0 ||
            req.tanh_inputs.size() != n ||
            req.hypot_a.size() != n ||
            req.hypot_b.size() != n ||
            req.interp_s.size() != n ||
            req.interp_x.size() != n ||
            req.interp_x0.size() != n ||
            req.interp_y0.size() != n ||
            req.interp_x1.size() != n ||
            req.interp_y1.size() != n ||
            req.dot3_a.size() != 3u * n ||
            req.dot3_b.size() != 3u * n ||
            req.ring_local.size() != 3u * n ||
            req.ring_rmat.size() != 9u) {
            out.error_message = "Input vector sizes mismatch in MetalParityRequest";
            return out;
        }

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

        id<MTLFunction> fn = [library newFunctionWithName:@"physics_core_parity_kernel"];
        if (fn == nil) {
            out.error_message = "Failed to find physics_core_parity_kernel in compiled MTLLibrary";
            return out;
        }

        id<MTLComputePipelineState> pso = [device newComputePipelineStateWithFunction:fn error:&err];
        if (pso == nil) {
            out.error_message = std::string("Metal pipeline creation error: ") +
                                (err != nil ? [[err localizedDescription] UTF8String] : "unknown");
            return out;
        }

        id<MTLCommandQueue> queue = [device newCommandQueue];
        if (queue == nil) {
            out.error_message = "Metal newCommandQueue failed";
            return out;
        }

        id<MTLBuffer> b_exp_in = make_float_buffer(device, req.exp_inputs);
        id<MTLBuffer> b_tanh_in = make_float_buffer(device, req.tanh_inputs);
        id<MTLBuffer> b_hypot_a = make_float_buffer(device, req.hypot_a);
        id<MTLBuffer> b_hypot_b = make_float_buffer(device, req.hypot_b);
        id<MTLBuffer> b_interp_s = make_float_buffer(device, req.interp_s);
        id<MTLBuffer> b_interp_x = make_float_buffer(device, req.interp_x);
        id<MTLBuffer> b_interp_x0 = make_float_buffer(device, req.interp_x0);
        id<MTLBuffer> b_interp_y0 = make_float_buffer(device, req.interp_y0);
        id<MTLBuffer> b_interp_x1 = make_float_buffer(device, req.interp_x1);
        id<MTLBuffer> b_interp_y1 = make_float_buffer(device, req.interp_y1);
        id<MTLBuffer> b_dot3_a = make_float_buffer(device, req.dot3_a);
        id<MTLBuffer> b_dot3_b = make_float_buffer(device, req.dot3_b);
        id<MTLBuffer> b_ring_local = make_float_buffer(device, req.ring_local);
        id<MTLBuffer> b_ring_rmat = make_float_buffer(device, req.ring_rmat);

        id<MTLBuffer> b_exp_out = make_empty_float_buffer(device, n);
        id<MTLBuffer> b_tanh_out = make_empty_float_buffer(device, n);
        id<MTLBuffer> b_hypot_out = make_empty_float_buffer(device, n);
        id<MTLBuffer> b_interp_out = make_empty_float_buffer(device, n);
        id<MTLBuffer> b_dot3_out = make_empty_float_buffer(device, n);
        id<MTLBuffer> b_ring_out = make_empty_float_buffer(device, 3u * n);
        id<MTLBuffer> b_meta = [device newBufferWithLength:sizeof(ParityMeta)
                                                  options:MTLResourceStorageModeShared];

        id<MTLCommandBuffer> cmd = [queue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
        [enc setComputePipelineState:pso];
        [enc setBuffer:b_exp_in offset:0 atIndex:0];
        [enc setBuffer:b_tanh_in offset:0 atIndex:1];
        [enc setBuffer:b_hypot_a offset:0 atIndex:2];
        [enc setBuffer:b_hypot_b offset:0 atIndex:3];
        [enc setBuffer:b_interp_s offset:0 atIndex:4];
        [enc setBuffer:b_interp_x offset:0 atIndex:5];
        [enc setBuffer:b_interp_x0 offset:0 atIndex:6];
        [enc setBuffer:b_interp_y0 offset:0 atIndex:7];
        [enc setBuffer:b_interp_x1 offset:0 atIndex:8];
        [enc setBuffer:b_interp_y1 offset:0 atIndex:9];
        [enc setBuffer:b_dot3_a offset:0 atIndex:10];
        [enc setBuffer:b_dot3_b offset:0 atIndex:11];
        [enc setBuffer:b_ring_local offset:0 atIndex:12];
        [enc setBuffer:b_ring_rmat offset:0 atIndex:13];
        [enc setBuffer:b_exp_out offset:0 atIndex:14];
        [enc setBuffer:b_tanh_out offset:0 atIndex:15];
        [enc setBuffer:b_hypot_out offset:0 atIndex:16];
        [enc setBuffer:b_interp_out offset:0 atIndex:17];
        [enc setBuffer:b_dot3_out offset:0 atIndex:18];
        [enc setBuffer:b_ring_out offset:0 atIndex:19];
        [enc setBuffer:b_meta offset:0 atIndex:20];
        const std::uint32_t n_u32 = static_cast<std::uint32_t>(n);
        [enc setBytes:&n_u32 length:sizeof(std::uint32_t) atIndex:21];

        const NSUInteger tg = 64;
        const NSUInteger grid = ((n + tg - 1u) / tg) * tg;
        [enc dispatchThreads:MTLSizeMake(grid, 1, 1)
       threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
        [enc endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];

        if ([cmd status] != MTLCommandBufferStatusCompleted) {
            NSError *cmd_err = [cmd error];
            out.error_message = std::string("Metal command buffer failed: ") +
                                (cmd_err != nil ? [[cmd_err localizedDescription] UTF8String] : "unknown");
            return out;
        }

        out.exp_out = read_float_buffer(b_exp_out, n);
        out.tanh_out = read_float_buffer(b_tanh_out, n);
        out.hypot_out = read_float_buffer(b_hypot_out, n);
        out.interp_out = read_float_buffer(b_interp_out, n);
        out.dot3_out = read_float_buffer(b_dot3_out, n);
        out.ring_out = read_float_buffer(b_ring_out, 3u * n);

        const ParityMeta meta = boundary_span(
            static_cast<const ParityMeta *>([b_meta contents]), 1u)[0];
        out.bike_real_1e_12 = meta.bike_real_1e_12;
        out.sizeof_tyre_model = meta.sizeof_tyre_model;
        out.sizeof_compliant_state = meta.sizeof_compliant_state;
        out.sizeof_station_state = meta.sizeof_station_state;
    }
    return out;
}

} // namespace bike::physics_core::test_metal
