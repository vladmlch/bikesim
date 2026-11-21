#include "test_sp3_rider_monitors_metal_helper.hpp"

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

namespace bike::physics_core::test_sp3b_metal {

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
        "efc_monitors.hpp"
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
kernel void sp3_trig_kernel(
    device const float *x_in [[buffer(0)]],
    device const float *y_in [[buffer(1)]],
    device float *atan_out [[buffer(2)]],
    device float *atan2_out [[buffer(3)]],
    device float *sin_out [[buffer(4)]],
    device float *cos_out [[buffer(5)]],
    constant uint &n [[buffer(6)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid >= n) {
        return;
    }
    const float x = x_in[gid];
    const float y = y_in[gid];
    atan_out[gid] = PortableTrig::atan_val<float, GpuToolkit>(x);
    atan2_out[gid] = PortableTrig::atan2_val<float, GpuToolkit>(y, x);
    sin_out[gid] = PortableTrig::sin_val<float, GpuToolkit>(x);
    cos_out[gid] = PortableTrig::cos_val<float, GpuToolkit>(x);
}

kernel void sp3_launch_kernel(
    constant bike::physics_core::LaunchParams<float> &cfg [[buffer(0)]],
    device const uchar *has_p [[buffer(1)]],
    device const bike::physics_core::PerceptsCore<float> *percepts [[buffer(2)]],
    device const bike::physics_core::ShiftFeedbackCore *shifts [[buffer(3)]],
    device const uchar *eligible [[buffer(4)]],
    device const uchar *ext_brake [[buffer(5)]],
    device bike::physics_core::LaunchCommandCore<float> *cmds_out [[buffer(6)]],
    device bike::physics_core::LaunchState<float> *state_inout [[buffer(7)]],
    constant uint &n_steps [[buffer(8)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    launch_reset<float>(state_inout);
    bool armed = false;
    ErrCode err = ErrCode::none;
    (void)launch_arm_start<float, GpuToolkit>(
        &cfg, launch_start_lean_unchecked<float, GpuToolkit>(&cfg, 0.0f),
        state_inout, armed, err);
    for (uint i = 0u; i < n_steps; ++i) {
        LaunchCommandCore<float> cmd{};
        (void)launch_update<float, GpuToolkit>(
            &cfg, has_p[i] != 0u, &percepts[i], shifts[i],
            eligible[i] != 0u, ext_brake[i] != 0u, state_inout, cmd, err);
        cmds_out[i] = cmd;
    }
}

kernel void sp3_front_lift_kernel(
    constant bike::physics_core::FrontLiftParams<float> &cfg [[buffer(0)]],
    device const uchar *has_p [[buffer(1)]],
    device const bike::physics_core::PerceptsCore<float> *percepts [[buffer(2)]],
    device bike::physics_core::ReactionCommandCore<float> *cmds_out [[buffer(3)]],
    device bike::physics_core::FrontLiftState<float> *state_inout [[buffer(4)]],
    constant uint &n_steps [[buffer(5)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    front_lift_reset<float>(state_inout);
    ErrCode err = ErrCode::none;
    for (uint i = 0u; i < n_steps; ++i) {
        ReactionCommandCore<float> cmd{};
        (void)front_lift_update<float, GpuToolkit>(
            &cfg, has_p[i] != 0u, &percepts[i], state_inout, cmd, err);
        cmds_out[i] = cmd;
    }
}

struct IntentRunScalars {
    int n_steps;
    int road_n;
    int section_n;
    bike::physics_core::SurfaceLabel default_surface;
    uchar pad[3];
    float lookahead_m;
    float spacing_m;
};

kernel void sp3_rider_intent_kernel(
    constant bike::physics_core::IntentParams<float> &icfg [[buffer(0)]],
    constant bike::physics_core::FrontLiftParams<float> &lcfg [[buffer(1)]],
    constant bike::physics_core::LaunchParams<float> &pcfg [[buffer(2)]],
    constant IntentRunScalars &sc [[buffer(3)]],
    device const float *road_x [[buffer(4)]],
    device const float *road_z [[buffer(5)]],
    device const bike::physics_core::SurfaceSectionCore<float> *sections [[buffer(6)]],
    device const bike::physics_core::IntentSignalsCore<float> *sigs [[buffer(7)]],
    device const float *wf_x [[buffer(8)]],
    device const float *wr_x [[buffer(9)]],
    device const float *speed [[buffer(10)]],
    device const int *rear_teeth [[buffer(11)]],
    device const float *hand_press [[buffer(12)]],
    device const bike::physics_core::LaunchContextCore *lctx [[buffer(13)]],
    device bike::physics_core::IntentResolutionCore<float> *res_out [[buffer(14)]],
    device float *posture_grade_out [[buffer(15)]],
    device float *preview_grade_out [[buffer(16)]],
    device bike::physics_core::RiderIntentState<float> *state_inout [[buffer(17)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    rider_intent_reset<float>(&icfg, state_inout);
    ErrCode err = ErrCode::none;
    bool armed = false;
    (void)rider_intent_arm_standstill_start<float, GpuToolkit>(
        &icfg, &pcfg,
        launch_start_lean_unchecked<float, GpuToolkit>(&pcfg, 0.0f),
        state_inout, armed, err);
    for (int i = 0; i < sc.n_steps; ++i) {
        float p_grade = 0.0f;
        float prev_grade = 0.0f;
        (void)eval_road_grades<float, GpuToolkit>(
            sc.road_n, road_x, road_z, wf_x[i], wr_x[i],
            sc.lookahead_m, sc.spacing_m, p_grade, prev_grade, err);
        posture_grade_out[i] = p_grade;
        preview_grade_out[i] = prev_grade;

        SurfaceLabel s_front = sc.default_surface;
        SurfaceLabel s_ahead = sc.default_surface;
        (void)surface_labels_at<float>(
            sc.default_surface, sc.section_n, sections, wf_x[i], s_front, err);
        (void)surface_labels_at<float>(
            sc.default_surface, sc.section_n, sections,
            wf_x[i] + sc.lookahead_m, s_ahead, err);

        RiderSensesCore<float> senses{};
        senses.hand_press_n = hand_press[i];
        senses.speed_mps = speed[i];
        senses.rear_teeth = rear_teeth[i];
        senses.surface_front = s_front;
        senses.surface_ahead = s_ahead;
        senses.grade_ahead = prev_grade;

        IntentControlCore<float> ctrl{};
        ctrl.rider_enabled = 1;
        IntentResolutionCore<float> res{};
        (void)rider_intent_resolve<float, GpuToolkit>(
            &icfg, &lcfg, &pcfg, ctrl, sigs[i], senses, i,
            p_grade, true, prev_grade, false, 0.0f, true, false, 0.0f,
            lctx[i], state_inout, res, err);
        res_out[i] = res;
    }
}

struct EfcRunScalars {
    int nefc;
    int efc_J_stride;
    int nbody;
    int ncon;
    int mon_steps;
};

kernel void sp3_efc_monitors_kernel(
    constant bike::physics_core::EfcScanConfig<float> &ecfg [[buffer(0)]],
    constant EfcRunScalars &sc [[buffer(1)]],
    device const int *efc_type [[buffer(2)]],
    device const int *efc_id [[buffer(3)]],
    device const float *efc_J [[buffer(4)]],
    device const float *efc_force [[buffer(5)]],
    device const float *body_xmat [[buffer(6)]],
    device const int *con_g1 [[buffer(7)]],
    device const int *con_g2 [[buffer(8)]],
    device const int *con_adr [[buffer(9)]],
    device const int *con_dim [[buffer(10)]],
    device bike::physics_core::EfcScanResult<float> *efc_out [[buffer(11)]],
    constant bike::physics_core::GroundedFilterConfig &gcfg [[buffer(12)]],
    constant bike::physics_core::BalanceMonitorConfig<float> &bcfg [[buffer(13)]],
    constant bike::physics_core::RollbackBrakeConfig<float> &rcfg [[buffer(14)]],
    device const uchar *mon_raw [[buffer(15)]],
    device const float *mon_speed [[buffer(16)]],
    device const float *mon_pos [[buffer(17)]],
    device const uchar *mon_cbrk [[buffer(18)]],
    device const uchar *mon_ren [[buffer(19)]],
    device uchar *mon_g_out [[buffer(20)]],
    device uchar *mon_b_out [[buffer(21)]],
    device float *mon_rb_out [[buffer(22)]],
    device bike::physics_core::BalanceMonitorState<float> *bal_final [[buffer(23)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    ErrCode err = ErrCode::none;
    EfcScanResult<float> scan{};
    (void)scan_efc_reactions<float, GpuToolkit>(
        &ecfg, sc.nefc, sc.efc_J_stride, sc.nbody,
        efc_type, efc_id, efc_J, efc_force, body_xmat, scan, err);
    (void)scan_contact_monitors<float, GpuToolkit>(
        &ecfg, sc.ncon, sc.nefc, con_g1, con_g2, con_adr, con_dim,
        efc_type, efc_id, efc_force, scan, err);
    *efc_out = scan;

    GroundedFilterState gf{};
    grounded_filter_reset(&gf);
    BalanceMonitorState<float> bm{};
    balance_monitor_reset<float>(&bcfg, &bm);
    StaticBrakeMonitorState<float> sb{};

    for (int i = 0; i < sc.mon_steps; ++i) {
        bool gval = false;
        (void)grounded_filter_update(&gcfg, mon_raw[i] != 0u, i, &gf, gval, err);
        mon_g_out[i] = gval ? 1u : 0u;

        float rb = 0.0f;
        (void)rollback_brake_demand<float, GpuToolkit>(
            &rcfg, mon_speed[i], mon_ren[i] != 0u, &sb, rb, err);
        mon_rb_out[i] = rb;

        BrakeRequestCore<float> breq{};
        breq.caller_front = mon_cbrk[i] != 0u ? 0.5f : 0.0f;
        breq.rollback_demand = rb;
        breq.caller_braking = mon_cbrk[i];
        breq.rider_present = 1;
        breq.rider_enabled = mon_ren[i];
        const BrakeResolutionCore<float> bres =
            resolve_brakes_core<float, GpuToolkit>(breq);

        const BalanceStopCore stop = balance_compute_stop<float, GpuToolkit>(
            bres.motor_braking != 0u, mon_speed[i], 0.25f, LaunchPhase::done, &bm);
        bool blost = false;
        (void)balance_monitor_update<float, GpuToolkit>(
            &bcfg, i, mon_pos[i], mon_speed[i], bres.riding != 0u, stop,
            &bm, blost, err);
        mon_b_out[i] = blost ? 1u : 0u;
    }
    *bal_final = bm;
}

kernel void sp3_torture_address_spaces_kernel(
    constant bike::physics_core::LaunchParams<float> &c_lcfg [[buffer(0)]],
    device const bike::physics_core::LaunchParams<float> *d_lcfg [[buffer(1)]],
    constant bike::physics_core::FrontLiftParams<float> &c_fcfg [[buffer(2)]],
    device const bike::physics_core::FrontLiftParams<float> *d_fcfg [[buffer(3)]],
    constant bike::physics_core::IntentParams<float> &c_icfg [[buffer(4)]],
    device const bike::physics_core::IntentParams<float> *d_icfg [[buffer(5)]],
    constant bike::physics_core::EfcScanConfig<float> &c_ecfg [[buffer(6)]],
    device const bike::physics_core::EfcScanConfig<float> *d_ecfg [[buffer(7)]],
    device bike::physics_core::Sp3PerWorldState<float> *d_state [[buffer(8)]],
    device float *out_vals [[buffer(9)]],
    uint gid [[thread_position_in_grid]]
) {
    using namespace bike::physics_core;
    if (gid != 0u) {
        return;
    }
    ErrCode err = ErrCode::none;
    LaunchParams<float> t_lcfg = c_lcfg;
    FrontLiftParams<float> t_fcfg = c_fcfg;
    IntentParams<float> t_icfg = c_icfg;
    EfcScanConfig<float> t_ecfg = c_ecfg;

    LaunchState<float> t_ls{};
    LaunchCommandCore<float> lcmd1{}, lcmd2{}, lcmd3{};
    PerceptsCore<float> p{};
    ShiftFeedbackCore sfb{};
    (void)launch_update<float, GpuToolkit>(&c_lcfg, true, &p, sfb, true, false, &t_ls, lcmd1, err);
    (void)launch_update<float, GpuToolkit>(d_lcfg, true, &p, sfb, true, false, &d_state->intent.launch, lcmd2, err);
    (void)launch_update<float, GpuToolkit>(&t_lcfg, true, &p, sfb, true, false, &t_ls, lcmd3, err);

    ReactionCommandCore<float> rcmd1{}, rcmd2{}, rcmd3{};
    (void)front_lift_update<float, GpuToolkit>(&c_fcfg, false, &p, &d_state->intent.reaction, rcmd1, err);
    (void)front_lift_update<float, GpuToolkit>(d_fcfg, false, &p, &d_state->intent.reaction, rcmd2, err);
    (void)front_lift_update<float, GpuToolkit>(&t_fcfg, false, &p, &d_state->intent.reaction, rcmd3, err);

    IntentControlCore<float> ctrl{};
    IntentSignalsCore<float> sig{};
    RiderSensesCore<float> sns{};
    LaunchContextCore lctx{};
    IntentResolutionCore<float> ires1{}, ires2{}, ires3{};
    (void)rider_intent_resolve<float, GpuToolkit>(
        &c_icfg, &c_fcfg, &c_lcfg, ctrl, sig, sns, 0, 0.0f, false, 0.0f,
        false, 0.0f, true, false, 0.0f, lctx, &d_state->intent, ires1, err);
    (void)rider_intent_resolve<float, GpuToolkit>(
        d_icfg, d_fcfg, d_lcfg, ctrl, sig, sns, 0, 0.0f, false, 0.0f,
        false, 0.0f, true, false, 0.0f, lctx, &d_state->intent, ires2, err);
    (void)rider_intent_resolve<float, GpuToolkit>(
        &t_icfg, &t_fcfg, &t_lcfg, ctrl, sig, sns, 0, 0.0f, false, 0.0f,
        false, 0.0f, true, false, 0.0f, lctx, &d_state->intent, ires3, err);

    int et[1] = {0};
    int eid[1] = {0};
    float ej[1] = {0.0f};
    float ef[1] = {0.0f};
    float xm[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    EfcScanResult<float> er1{}, er2{}, er3{};
    (void)scan_efc_reactions<float, GpuToolkit>(&c_ecfg, 0, 1, 1, et, eid, ej, ef, xm, er1, err);
    (void)scan_efc_reactions<float, GpuToolkit>(d_ecfg, 0, 1, 1, et, eid, ej, ef, xm, er2, err);
    (void)scan_efc_reactions<float, GpuToolkit>(&t_ecfg, 0, 1, 1, et, eid, ej, ef, xm, er3, err);

    out_vals[0] = lcmd1.effort_nm + lcmd2.effort_nm + lcmd3.effort_nm +
                  rcmd1.effort_factor + rcmd2.effort_factor + rcmd3.effort_factor +
                  ires1.control.human_torque_nm + ires2.control.human_torque_nm + ires3.control.human_torque_nm +
                  er1.hand_press_n + er2.hand_press_n + er3.hand_press_n;
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

template <typename T>
void read_struct_buffer_into(id<MTLBuffer> buf, T &dst) {
    const std::span<const T> src = boundary_span(
        static_cast<const T *>([buf contents]), 1u);
    dst = src[0];
}

struct IntentRunScalarsHost {
    i32 n_steps;
    i32 road_n;
    i32 section_n;
    SurfaceLabel default_surface;
    u8 pad[3];
    float lookahead_m;
    float spacing_m;
};

struct EfcRunScalarsHost {
    i32 nefc;
    i32 efc_J_stride;
    i32 nbody;
    i32 ncon;
    i32 mon_steps;
};

} // namespace

std::unique_ptr<MetalSp3Output> run_metal_sp3_parity(const MetalSp3Request &req) {
    auto out = std::make_unique<MetalSp3Output>();
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

        // Verify MSL < 3.2 compilation (MTLLanguageVersion3_0 and MTLLanguageVersion3_1)
        // matching warp-metal's wp_metal_load_library target, including constant/device/thread
        // address-space instantiations in sp3_torture_address_spaces_kernel.
        for (const MTLLanguageVersion ver : {MTLLanguageVersion3_0, MTLLanguageVersion3_1}) {
            MTLCompileOptions *ver_opts = [[MTLCompileOptions alloc] init];
            ver_opts.mathMode = MTLMathModeSafe;
            ver_opts.mathFloatingPointFunctions = MTLMathFloatingPointFunctionsPrecise;
            ver_opts.languageVersion = ver;
            NSError *ver_err = nil;
            id<MTLLibrary> ver_lib = [device newLibraryWithSource:ns_src options:ver_opts error:&ver_err];
            if (ver_lib == nil) {
                out->error_message = std::string("Metal MSL 3.x compile error: ") +
                                     (ver_err != nil ? [[ver_err localizedDescription] UTF8String] : "unknown");
                return out;
            }
            id<MTLFunction> torture_fn = [ver_lib newFunctionWithName:@"sp3_torture_address_spaces_kernel"];
            if (torture_fn == nil) {
                out->error_message = "Missing sp3_torture_address_spaces_kernel in MSL 3.x library";
                return out;
            }
            id<MTLComputePipelineState> torture_pso =
                [device newComputePipelineStateWithFunction:torture_fn error:&ver_err];
            if (torture_pso == nil) {
                out->error_message = std::string("Metal MSL 3.x torture PSO error: ") +
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

        id<MTLComputePipelineState> pso_trig = make_pso(@"sp3_trig_kernel");
        id<MTLComputePipelineState> pso_launch = make_pso(@"sp3_launch_kernel");
        id<MTLComputePipelineState> pso_lift = make_pso(@"sp3_front_lift_kernel");
        id<MTLComputePipelineState> pso_intent = make_pso(@"sp3_rider_intent_kernel");
        id<MTLComputePipelineState> pso_efc = make_pso(@"sp3_efc_monitors_kernel");
        if (pso_trig == nil || pso_launch == nil || pso_lift == nil ||
            pso_intent == nil || pso_efc == nil) {
            return out;
        }

        id<MTLCommandQueue> queue = [device newCommandQueue];
        if (queue == nil) {
            out->error_message = "Metal newCommandQueue failed";
            return out;
        }

        id<MTLCommandBuffer> cmd = [queue commandBuffer];
        if (cmd == nil) {
            out->error_message = "Metal commandBuffer failed";
            return out;
        }

        // 1. Trig buffers
        const std::size_t n_trig = req.trig_x.size();
        id<MTLBuffer> b_tx = make_vec_buffer(device, req.trig_x);
        id<MTLBuffer> b_ty = make_vec_buffer(device, req.trig_y);
        id<MTLBuffer> b_atan = make_empty_buffer<float>(device, n_trig);
        id<MTLBuffer> b_atan2 = make_empty_buffer<float>(device, n_trig);
        id<MTLBuffer> b_sin = make_empty_buffer<float>(device, n_trig);
        id<MTLBuffer> b_cos = make_empty_buffer<float>(device, n_trig);
        const std::uint32_t u_trig = static_cast<std::uint32_t>(n_trig);
        if (n_trig > 0) {
            id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
            [enc setComputePipelineState:pso_trig];
            [enc setBuffer:b_tx offset:0 atIndex:0];
            [enc setBuffer:b_ty offset:0 atIndex:1];
            [enc setBuffer:b_atan offset:0 atIndex:2];
            [enc setBuffer:b_atan2 offset:0 atIndex:3];
            [enc setBuffer:b_sin offset:0 atIndex:4];
            [enc setBuffer:b_cos offset:0 atIndex:5];
            [enc setBytes:&u_trig length:sizeof(u_trig) atIndex:6];
            const NSUInteger tg = std::min<NSUInteger>(64u, n_trig);
            [enc dispatchThreads:MTLSizeMake(n_trig, 1, 1)
                threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
            [enc endEncoding];
        }

        // 2. Launch buffers
        const std::size_t n_launch = req.launch_has_percepts.size();
        id<MTLBuffer> b_lcfg = make_struct_buffer(device, req.launch_cfg);
        id<MTLBuffer> b_lhp = make_vec_buffer(device, req.launch_has_percepts);
        id<MTLBuffer> b_lp = make_vec_buffer(device, req.launch_percepts);
        id<MTLBuffer> b_lsh = make_vec_buffer(device, req.launch_shift);
        id<MTLBuffer> b_lel = make_vec_buffer(device, req.launch_eligible);
        id<MTLBuffer> b_leb = make_vec_buffer(device, req.launch_ext_brake);
        id<MTLBuffer> b_lout = make_empty_buffer<LaunchCommandCore<float>>(device, n_launch);
        id<MTLBuffer> b_lst = make_empty_buffer<LaunchState<float>>(device, 1u);
        const std::uint32_t u_launch = static_cast<std::uint32_t>(n_launch);
        if (n_launch > 0) {
            id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
            [enc setComputePipelineState:pso_launch];
            [enc setBuffer:b_lcfg offset:0 atIndex:0];
            [enc setBuffer:b_lhp offset:0 atIndex:1];
            [enc setBuffer:b_lp offset:0 atIndex:2];
            [enc setBuffer:b_lsh offset:0 atIndex:3];
            [enc setBuffer:b_lel offset:0 atIndex:4];
            [enc setBuffer:b_leb offset:0 atIndex:5];
            [enc setBuffer:b_lout offset:0 atIndex:6];
            [enc setBuffer:b_lst offset:0 atIndex:7];
            [enc setBytes:&u_launch length:sizeof(u_launch) atIndex:8];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
                threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
            [enc endEncoding];
        }

        // 3. FrontLift buffers
        const std::size_t n_lift = req.lift_has_percepts.size();
        id<MTLBuffer> b_fcfg = make_struct_buffer(device, req.lift_cfg);
        id<MTLBuffer> b_fhp = make_vec_buffer(device, req.lift_has_percepts);
        id<MTLBuffer> b_fp = make_vec_buffer(device, req.lift_percepts);
        id<MTLBuffer> b_fout = make_empty_buffer<ReactionCommandCore<float>>(device, n_lift);
        id<MTLBuffer> b_fst = make_empty_buffer<FrontLiftState<float>>(device, 1u);
        const std::uint32_t u_lift = static_cast<std::uint32_t>(n_lift);
        if (n_lift > 0) {
            id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
            [enc setComputePipelineState:pso_lift];
            [enc setBuffer:b_fcfg offset:0 atIndex:0];
            [enc setBuffer:b_fhp offset:0 atIndex:1];
            [enc setBuffer:b_fp offset:0 atIndex:2];
            [enc setBuffer:b_fout offset:0 atIndex:3];
            [enc setBuffer:b_fst offset:0 atIndex:4];
            [enc setBytes:&u_lift length:sizeof(u_lift) atIndex:5];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
                threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
            [enc endEncoding];
        }

        // 4. RiderIntent buffers
        const std::size_t n_intent = req.intent_signals.size();
        const IntentRunScalarsHost iscalars{
            .n_steps = static_cast<i32>(n_intent),
            .road_n = static_cast<i32>(req.road_x.size()),
            .section_n = static_cast<i32>(req.surface_sections.size()),
            .default_surface = req.default_surface,
            .pad = {0, 0, 0},
            .lookahead_m = req.road_lookahead_m,
            .spacing_m = req.road_spacing_m,
        };
        id<MTLBuffer> b_icfg = make_struct_buffer(device, req.intent_cfg);
        id<MTLBuffer> b_ilcfg = make_struct_buffer(device, req.intent_lift_cfg);
        id<MTLBuffer> b_ipcfg = make_struct_buffer(device, req.intent_launch_cfg);
        id<MTLBuffer> b_rx = make_vec_buffer(device, req.road_x);
        id<MTLBuffer> b_rz = make_vec_buffer(device, req.road_z);
        id<MTLBuffer> b_sec = make_vec_buffer(device, req.surface_sections);
        id<MTLBuffer> b_isig = make_vec_buffer(device, req.intent_signals);
        id<MTLBuffer> b_wfx = make_vec_buffer(device, req.wheel_front_x);
        id<MTLBuffer> b_wrx = make_vec_buffer(device, req.wheel_rear_x);
        id<MTLBuffer> b_ispd = make_vec_buffer(device, req.speed_mps);
        id<MTLBuffer> b_itth = make_vec_buffer(device, req.rear_teeth);
        id<MTLBuffer> b_ihp = make_vec_buffer(device, req.hand_press_n);
        id<MTLBuffer> b_ilctx = make_vec_buffer(device, req.intent_launch_ctx);
        id<MTLBuffer> b_iout = make_empty_buffer<IntentResolutionCore<float>>(device, n_intent);
        id<MTLBuffer> b_pg = make_empty_buffer<float>(device, n_intent);
        id<MTLBuffer> b_pvg = make_empty_buffer<float>(device, n_intent);
        id<MTLBuffer> b_ist = make_empty_buffer<RiderIntentState<float>>(device, 1u);
        if (n_intent > 0) {
            id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
            [enc setComputePipelineState:pso_intent];
            [enc setBuffer:b_icfg offset:0 atIndex:0];
            [enc setBuffer:b_ilcfg offset:0 atIndex:1];
            [enc setBuffer:b_ipcfg offset:0 atIndex:2];
            [enc setBytes:&iscalars length:sizeof(iscalars) atIndex:3];
            [enc setBuffer:b_rx offset:0 atIndex:4];
            [enc setBuffer:b_rz offset:0 atIndex:5];
            [enc setBuffer:b_sec offset:0 atIndex:6];
            [enc setBuffer:b_isig offset:0 atIndex:7];
            [enc setBuffer:b_wfx offset:0 atIndex:8];
            [enc setBuffer:b_wrx offset:0 atIndex:9];
            [enc setBuffer:b_ispd offset:0 atIndex:10];
            [enc setBuffer:b_itth offset:0 atIndex:11];
            [enc setBuffer:b_ihp offset:0 atIndex:12];
            [enc setBuffer:b_ilctx offset:0 atIndex:13];
            [enc setBuffer:b_iout offset:0 atIndex:14];
            [enc setBuffer:b_pg offset:0 atIndex:15];
            [enc setBuffer:b_pvg offset:0 atIndex:16];
            [enc setBuffer:b_ist offset:0 atIndex:17];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
                threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
            [enc endEncoding];
        }

        // 5. EFC & Monitors buffers
        const std::size_t n_mon = req.mon_raw_grounded.size();
        const EfcRunScalarsHost escalars{
            .nefc = req.nefc,
            .efc_J_stride = req.efc_J_stride,
            .nbody = req.nbody,
            .ncon = req.ncon,
            .mon_steps = static_cast<i32>(n_mon),
        };
        id<MTLBuffer> b_ecfg = make_struct_buffer(device, req.efc_cfg);
        id<MTLBuffer> b_etype = make_vec_buffer(device, req.efc_type);
        id<MTLBuffer> b_eid = make_vec_buffer(device, req.efc_id);
        id<MTLBuffer> b_ej = make_vec_buffer(device, req.efc_J);
        id<MTLBuffer> b_ef = make_vec_buffer(device, req.efc_force);
        id<MTLBuffer> b_xm = make_vec_buffer(device, req.body_xmat);
        id<MTLBuffer> b_cg1 = make_vec_buffer(device, req.contact_geom1);
        id<MTLBuffer> b_cg2 = make_vec_buffer(device, req.contact_geom2);
        id<MTLBuffer> b_cadr = make_vec_buffer(device, req.contact_efc_address);
        id<MTLBuffer> b_cdim = make_vec_buffer(device, req.contact_dim);
        id<MTLBuffer> b_eout = make_empty_buffer<EfcScanResult<float>>(device, 1u);
        id<MTLBuffer> b_gcfg = make_struct_buffer(device, req.grounded_cfg);
        id<MTLBuffer> b_bcfg = make_struct_buffer(device, req.balance_cfg);
        id<MTLBuffer> b_rcfg = make_struct_buffer(device, req.rollback_cfg);
        id<MTLBuffer> b_mraw = make_vec_buffer(device, req.mon_raw_grounded);
        id<MTLBuffer> b_mspd = make_vec_buffer(device, req.mon_speed_mps);
        id<MTLBuffer> b_mpos = make_vec_buffer(device, req.mon_pos_m);
        id<MTLBuffer> b_mcbrk = make_vec_buffer(device, req.mon_caller_braking);
        id<MTLBuffer> b_mren = make_vec_buffer(device, req.mon_rider_enabled);
        id<MTLBuffer> b_mgout = make_empty_buffer<u8>(device, n_mon);
        id<MTLBuffer> b_mbout = make_empty_buffer<u8>(device, n_mon);
        id<MTLBuffer> b_mrbout = make_empty_buffer<float>(device, n_mon);
        id<MTLBuffer> b_bfinal = make_empty_buffer<BalanceMonitorState<float>>(device, 1u);
        {
            id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
            [enc setComputePipelineState:pso_efc];
            [enc setBuffer:b_ecfg offset:0 atIndex:0];
            [enc setBytes:&escalars length:sizeof(escalars) atIndex:1];
            [enc setBuffer:b_etype offset:0 atIndex:2];
            [enc setBuffer:b_eid offset:0 atIndex:3];
            [enc setBuffer:b_ej offset:0 atIndex:4];
            [enc setBuffer:b_ef offset:0 atIndex:5];
            [enc setBuffer:b_xm offset:0 atIndex:6];
            [enc setBuffer:b_cg1 offset:0 atIndex:7];
            [enc setBuffer:b_cg2 offset:0 atIndex:8];
            [enc setBuffer:b_cadr offset:0 atIndex:9];
            [enc setBuffer:b_cdim offset:0 atIndex:10];
            [enc setBuffer:b_eout offset:0 atIndex:11];
            [enc setBuffer:b_gcfg offset:0 atIndex:12];
            [enc setBuffer:b_bcfg offset:0 atIndex:13];
            [enc setBuffer:b_rcfg offset:0 atIndex:14];
            [enc setBuffer:b_mraw offset:0 atIndex:15];
            [enc setBuffer:b_mspd offset:0 atIndex:16];
            [enc setBuffer:b_mpos offset:0 atIndex:17];
            [enc setBuffer:b_mcbrk offset:0 atIndex:18];
            [enc setBuffer:b_mren offset:0 atIndex:19];
            [enc setBuffer:b_mgout offset:0 atIndex:20];
            [enc setBuffer:b_mbout offset:0 atIndex:21];
            [enc setBuffer:b_mrbout offset:0 atIndex:22];
            [enc setBuffer:b_bfinal offset:0 atIndex:23];
            [enc dispatchThreads:MTLSizeMake(1, 1, 1)
                threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
            [enc endEncoding];
        }

        [cmd commit];
        [cmd waitUntilCompleted];
        if ([cmd status] != MTLCommandBufferStatusCompleted) {
            NSError *cmdErr = [cmd error];
            out->error_message = std::string("Metal command failed: ") +
                                 (cmdErr != nil ? [[cmdErr localizedDescription] UTF8String] : "unknown");
            return out;
        }

        out->atan_out = read_vec_buffer<float>(b_atan, n_trig);
        out->atan2_out = read_vec_buffer<float>(b_atan2, n_trig);
        out->sin_out = read_vec_buffer<float>(b_sin, n_trig);
        out->cos_out = read_vec_buffer<float>(b_cos, n_trig);

        out->launch_out = read_vec_buffer<LaunchCommandCore<float>>(b_lout, n_launch);
        read_struct_buffer_into(b_lst, out->launch_final_state);

        out->lift_out = read_vec_buffer<ReactionCommandCore<float>>(b_fout, n_lift);
        read_struct_buffer_into(b_fst, out->lift_final_state);

        out->intent_out = read_vec_buffer<IntentResolutionCore<float>>(b_iout, n_intent);
        out->road_posture_grade = read_vec_buffer<float>(b_pg, n_intent);
        out->road_preview_grade = read_vec_buffer<float>(b_pvg, n_intent);
        read_struct_buffer_into(b_ist, out->intent_final_state);

        read_struct_buffer_into(b_eout, out->efc_out);
        out->mon_grounded_out = read_vec_buffer<u8>(b_mgout, n_mon);
        out->mon_balance_lost_out = read_vec_buffer<u8>(b_mbout, n_mon);
        out->mon_rollback_demand_out = read_vec_buffer<float>(b_mrbout, n_mon);
        read_struct_buffer_into(b_bfinal, out->mon_balance_final);
    }
    return out;
}

} // namespace bike::physics_core::test_sp3b_metal
