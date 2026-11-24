// native/tests/gpu/test_gpu_model_geometry.cpp — C++23 drift guard for the hardcoded model
// geometry and index constants in native/src/gpu/common.hpp.
//
// Every reference value is taken from MuJoCo (mj_loadXML, mj_name2id, mj_kinematics,
// mj_comPos, mj_applyFT) using the model XML, never from the constants under test. The unit
// tests that exercise the GPU helpers compare them against the same constants, so they cannot
// notice when model.xml and common.hpp disagree; this test can.
//
// Usage: bike_test_gpu_model_geometry [model.xml] [case-name|--list]
//        (an argument ending in ".xml" selects the model file; without one the model is
//         located relative to BIKE_REPO_ROOT or the cwd)

#include "../../src/config_types.hpp"
#include "../../src/diag.hpp"
#include "../../src/gpu/common.hpp"
#include "../../src/physics_core/cpu_exact_toolkit.hpp"
#include "../../src/physics_core/gpu_toolkit.hpp"
#include "../../src/physics_core/kinematics.hpp"
#include "../../src/physics_core/tyre_upload.hpp"
#include "../../src/platform/argv.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <format>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

using bike::gpu::GpuModelIndices;
using bike::gpu::GpuVec2;
using bike::gpu::kNumRiderJoints;

struct Fixture;
using TestFunction = void (*)(const Fixture &);

struct TestCase {
    std::string_view name;
    TestFunction run;
};

void require(bool condition, std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string(message));
    }
}

// GpuModelIndices addresses and MuJoCo ids are int for the kernel ABI;
// container subscripts take size_t, so the explicit conversion lives in one
// place (same helper as the other gpu tests).
constexpr std::size_t to_index(int field) {
    return static_cast<std::size_t>(field);
}

// The single raw mjModel/mjData pointer -> std::span boundary. MuJoCo owns
// these arrays for the model/data lifetime and sizes them by the adjacent
// counts (njnt, ngeom, nq, nv, nbody); every access range in this file is
// wrapped once through this helper. The (pointer, count) construction itself
// is -Wunsafe-buffer-usage-in-container — suppressed here, scoped exactly like
// native/src/platform/argv.hpp and native/src/gpu/model_assets.hpp's
// detail::model_span (the header is not included just for this helper).
template <typename T>
[[nodiscard]] std::span<T> model_span(T *ptr, std::size_t count) noexcept {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    return std::span<T>(ptr, count);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
}

// 3-strided row view (xpos/xanchor/xaxis/geom_size); subspan is the sanctioned
// replacement for the raw 3*id+comp pointer arithmetic per the Tasks 10-16
// recipe.
std::span<const mjtNum> row3(std::span<const mjtNum> values, int row) {
    return values.subspan(3 * to_index(row), 3);
}

static_assert(sizeof(GpuModelIndices) == 240, "GpuModelIndices must be 240 bytes");

constexpr const char *kRelModelPath = "tests/fixtures/model/welded.xml";

// Joint names for the rider slots, in the same order as GpuModelIndices::rider_qpos/rider_dof.
constexpr std::array<const char *, kNumRiderJoints> kRiderJointNames = {
    "rider_hip_front",
    "rider_knee_front",
    "rider_hip_rear",
    "rider_knee_rear",
    "rider_torso_hinge",
    "rider_shoulder_left",
    "rider_elbow_left",
    "rider_shoulder_right",
    "rider_elbow_right",
};

// Tolerances. Float32 rounding of order-1 m coordinates is about 6e-8 m, so the bounds below sit
// far above float32 noise while still resolving sub-millimetre constant errors.
constexpr double kAxisTol = 1e-4;        // unit-axis components (dimensionless)
constexpr double kAnchorTolM = 1e-4;     // world anchor coordinates (m)
constexpr double kQfrcRelTol = 1e-4;     // relative to |F| * kLeverBoundM
constexpr double kLeverBoundM = 1.5;     // upper bound on contact-to-joint lever arm (model: ~1.2 m max)
constexpr double kQfrcAbsFloorNm = 1e-6; // N*m floor so that |F| ~ 0 does not produce a zero bound

constexpr int kMaxReportedPerCheck = 8;

struct CheckStats {
    std::string name;
    int samples = 0;
    int failures = 0;
    double max_err = 0.0;   // max |error|, in the units of the check
    double max_ratio = 0.0; // max |error| / tolerance; a check passes iff this is <= 1
};

// Per-case collector. Each record() adds one sample to the row named `check`;
// mismatches are printed to stderr (first kMaxReportedPerCheck per row) and
// counted, so one case run reports all of its drifted constants at once.
struct CheckTable {
    std::vector<CheckStats> rows;

    CheckStats &row_for(std::string_view check) {
        for (CheckStats &s : rows) {
            if (s.name == check) {
                return s;
            }
        }
        rows.push_back(CheckStats{.name = std::string(check)});
        return rows.back();
    }

    void record(std::string_view check, double err, double tol, std::string_view ctx) {
        CheckStats &s = row_for(check);
        if (!std::isfinite(err)) {
            err = std::numeric_limits<double>::infinity();
        }
        ++s.samples;
        s.max_err = std::max(s.max_err, err);
        const double ratio =
            tol > 0.0 ? err / tol : (err > 0.0 ? std::numeric_limits<double>::infinity() : 0.0);
        s.max_ratio = std::max(s.max_ratio, ratio);
        if (!(err <= tol)) {
            if (s.failures < kMaxReportedPerCheck) {
                std::cerr << std::format("MISMATCH [{}] {} |err|={:.9g} tol={:.9g}\n",
                                         check, ctx, err, tol);
            }
            ++s.failures;
        }
    }

    [[nodiscard]] int total_samples() const {
        int total = 0;
        for (const CheckStats &s : rows) {
            total += s.samples;
        }
        return total;
    }

    [[nodiscard]] int total_failures() const {
        int total = 0;
        for (const CheckStats &s : rows) {
            total += s.failures;
        }
        return total;
    }

    // Same columns as the old monolithic report; one table per case.
    void print() const {
        std::cout << std::format("{:<44} {:>8} {:>9} {:>14} {:>14}\n",
                                 "check", "samples", "failures", "max_abs_err", "max_err/tol");
        for (const CheckStats &s : rows) {
            std::cout << std::format("{:<44} {:>8} {:>9} {:>14.6e} {:>14.6e}\n",
                                     s.name, s.samples, s.failures, s.max_err, s.max_ratio);
        }
    }
};

// Prints the case's table (same as the old unconditional report) and turns an
// accumulated mismatch count into the case's failure.
void finish_checks(const CheckTable &table) {
    table.print();
    const int failures = table.total_failures();
    require(failures == 0,
            std::format("{} mismatch(es) between common.hpp geometry/index constants and model.xml "
                        "(see MISMATCH lines above)", failures));
}

// ---------------------------------------------------------------------------------------------
// Model entity resolution. All names come from model.xml; nothing here reads the constants.
// ---------------------------------------------------------------------------------------------

int named(const mjModel *m, mjtObj type, std::string_view type_label, const std::string &name) {
    const int id = mj_name2id(m, type, name.c_str());
    require(id >= 0, "model has no " + std::string(type_label) + " named '" + name + "'");
    return id;
}

struct ModelIds {
    // Bodies
    int frame = -1;
    int steer = -1;
    int fork_lower = -1;
    int front_wheel = -1;
    int chainstay = -1;
    int seatstay = -1;
    int rear_wheel = -1;
    // Joints
    int root_x = -1;
    int root_z = -1;
    int root_pitch = -1;
    int crank_spin = -1;
    int fork_travel = -1;
    int front_wheel_spin = -1;
    int rear_wheel_spin = -1;
    int shock_stroke = -1;
    int drive_shaft_spin = -1;
    int main_pivot = -1;
    int horst_pivot = -1;
    std::array<int, kNumRiderJoints> rider_joint{};
    // Actuators
    std::array<int, kNumRiderJoints> rider_act{};
    // Geoms
    int front_tire = -1;
    int rear_tire = -1;
    int front_contact = -1;
    int rear_contact = -1;
    // Tendons
    int freehub = -1;
    int crank_clutch = -1;
};

ModelIds resolve_ids(const mjModel *m) {
    ModelIds ids;
    ids.frame = named(m, mjOBJ_BODY, "body", "frame");
    ids.steer = named(m, mjOBJ_BODY, "body", "steer");
    ids.fork_lower = named(m, mjOBJ_BODY, "body", "fork_lower");
    ids.front_wheel = named(m, mjOBJ_BODY, "body", "front_wheel");
    ids.chainstay = named(m, mjOBJ_BODY, "body", "chainstay");
    ids.seatstay = named(m, mjOBJ_BODY, "body", "seatstay");
    ids.rear_wheel = named(m, mjOBJ_BODY, "body", "rear_wheel");

    ids.root_x = named(m, mjOBJ_JOINT, "joint", "root_x");
    ids.root_z = named(m, mjOBJ_JOINT, "joint", "root_z");
    ids.root_pitch = named(m, mjOBJ_JOINT, "joint", "root_pitch");
    ids.crank_spin = named(m, mjOBJ_JOINT, "joint", "crank_spin");
    ids.fork_travel = named(m, mjOBJ_JOINT, "joint", "fork_travel");
    ids.front_wheel_spin = named(m, mjOBJ_JOINT, "joint", "front_wheel_spin");
    ids.rear_wheel_spin = named(m, mjOBJ_JOINT, "joint", "rear_wheel_spin");
    ids.shock_stroke = named(m, mjOBJ_JOINT, "joint", "shock_stroke");
    ids.drive_shaft_spin = named(m, mjOBJ_JOINT, "joint", "drive_shaft_spin");
    ids.main_pivot = named(m, mjOBJ_JOINT, "joint", "main_pivot");
    ids.horst_pivot = named(m, mjOBJ_JOINT, "joint", "horst_pivot");
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const std::string joint = kRiderJointNames[to_index(j)];
        ids.rider_joint[to_index(j)] = named(m, mjOBJ_JOINT, "joint", joint);
        ids.rider_act[to_index(j)] = named(m, mjOBJ_ACTUATOR, "actuator", "act_" + joint);
    }

    ids.front_tire = named(m, mjOBJ_GEOM, "geom", "geom_front_tire");
    ids.rear_tire = named(m, mjOBJ_GEOM, "geom", "geom_rear_tire");
    ids.front_contact = named(m, mjOBJ_GEOM, "geom", "geom_front_contact");
    ids.rear_contact = named(m, mjOBJ_GEOM, "geom", "geom_rear_contact");
    ids.freehub = named(m, mjOBJ_TENDON, "tendon", "ideal_mid_drive_freehub");
    ids.crank_clutch = named(m, mjOBJ_TENDON, "tendon", "crank_clutch");
    return ids;
}

// ---------------------------------------------------------------------------------------------
// Loaded model plus sized views of its arrays. Counts are narrowed once from
// mjtSize; check_indices pins them equal to the GpuModelIndices constants, so
// int carries every model this test covers.
// ---------------------------------------------------------------------------------------------

using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

struct ModelCounts {
    int nq = 0;
    int nv = 0;
    int nu = 0;
    int nbody = 0;
    int njnt = 0;
    int ngeom = 0;
};

// Read-only views of the mjModel arrays this test reads (jnt_range is
// 2*njnt, geom_size 3*ngeom, qpos0 nq).
struct ModelViews {
    std::span<const int> jnt_qposadr;
    std::span<const int> jnt_dofadr;
    std::span<const mjtBool> jnt_limited;
    std::span<const mjtNum> jnt_range;
    std::span<const mjtNum> geom_size;
    std::span<const mjtNum> qpos0;
};

// Views of the mjData arrays; qpos/qvel are mutable for pose staging, the
// kinematics outputs are read-only.
struct DataViews {
    std::span<mjtNum> qpos;
    std::span<mjtNum> qvel;
    std::span<const mjtNum> xpos;
    std::span<const mjtNum> xanchor;
    std::span<const mjtNum> xaxis;
};

struct Fixture {
    ModelPtr m;
    DataPtr d;
    ModelCounts counts;
    ModelViews mv;
    DataViews dv;
    ModelIds ids;
    GpuModelIndices idx{};
};

ModelPtr load_model(const std::filesystem::path &xml) {
    std::array<char, 1024> error{};
    mjModel *raw = mj_loadXML(xml.string().c_str(), nullptr, error.data(),
                              static_cast<int>(error.size()));
    require(raw != nullptr, "mj_loadXML(" + xml.string() + "): " + error.data());
    return {raw, &mj_deleteModel};
}

Fixture make_fixture(const std::filesystem::path &xml) {
    Fixture f{
        .m = load_model(xml),
        .d = DataPtr(nullptr, &mj_deleteData),
        .counts = {},
        .mv = {},
        .dv = {},
        .ids = {},
        .idx = {},
    };
    const mjModel *m = f.m.get();
    f.ids = resolve_ids(m);
    f.counts = ModelCounts{
        .nq = static_cast<int>(m->nq),
        .nv = static_cast<int>(m->nv),
        .nu = static_cast<int>(m->nu),
        .nbody = static_cast<int>(m->nbody),
        .njnt = static_cast<int>(m->njnt),
        .ngeom = static_cast<int>(m->ngeom),
    };
    const std::size_t njnt = to_index(f.counts.njnt);
    f.mv = ModelViews{
        .jnt_qposadr = model_span(m->jnt_qposadr, njnt),
        .jnt_dofadr = model_span(m->jnt_dofadr, njnt),
        .jnt_limited = model_span(m->jnt_limited, njnt),
        .jnt_range = model_span(m->jnt_range, 2 * njnt),
        .geom_size = model_span(m->geom_size, 3 * to_index(f.counts.ngeom)),
        .qpos0 = model_span(m->qpos0, to_index(f.counts.nq)),
    };
    f.d = DataPtr(mj_makeData(m), &mj_deleteData);
    require(f.d != nullptr, "mj_makeData returned a null mjData");
    const mjData *d = f.d.get();
    const std::size_t nbody3 = 3 * to_index(f.counts.nbody);
    f.dv = DataViews{
        .qpos = model_span(d->qpos, to_index(f.counts.nq)),
        .qvel = model_span(d->qvel, to_index(f.counts.nv)),
        .xpos = model_span(d->xpos, nbody3),
        .xanchor = model_span(d->xanchor, 3 * njnt),
        .xaxis = model_span(d->xaxis, 3 * njnt),
    };
    return f;
}

// ---------------------------------------------------------------------------------------------
// Index table: every default field of GpuModelIndices{} that names a model entity.
// ---------------------------------------------------------------------------------------------

void check_index(CheckTable &table, std::string_view field, std::string_view entity, int constant,
                 int model_value) {
    table.record("indices", std::fabs(static_cast<double>(constant - model_value)), 0.0,
                 std::format("{} ({}): constant={} model={}", field, entity, constant, model_value));
}

void check_indices(const Fixture &f, CheckTable &table) {
    check_index(table, "nq", "model nq", f.idx.nq, f.counts.nq);
    check_index(table, "nv", "model nv", f.idx.nv, f.counts.nv);
    check_index(table, "nu", "model nu", f.idx.nu, f.counts.nu);
    check_index(table, "nbody", "model nbody", f.idx.nbody, f.counts.nbody);

    check_index(table, "frame_body", "body frame", f.idx.frame_body, f.ids.frame);
    check_index(table, "steer_body", "body steer", f.idx.steer_body, f.ids.steer);
    check_index(table, "fork_lower_body", "body fork_lower", f.idx.fork_lower_body, f.ids.fork_lower);
    check_index(table, "front_wheel_body", "body front_wheel", f.idx.front_wheel_body, f.ids.front_wheel);
    check_index(table, "chainstay_body", "body chainstay", f.idx.chainstay_body, f.ids.chainstay);
    check_index(table, "seatstay_body", "body seatstay", f.idx.seatstay_body, f.ids.seatstay);
    check_index(table, "rear_wheel_body", "body rear_wheel", f.idx.rear_wheel_body, f.ids.rear_wheel);

    struct JointEntry {
        std::string_view qpos_field;
        std::string_view dof_field;
        std::string_view joint;
        int qpos_const;
        int dof_const;
        int joint_id;
    };
    const std::array<JointEntry, 9> joints{{
        {.qpos_field = "root_x_qpos", .dof_field = "root_x_dof", .joint = "root_x",
         .qpos_const = f.idx.root_x_qpos, .dof_const = f.idx.root_x_dof, .joint_id = f.ids.root_x},
        {.qpos_field = "root_z_qpos", .dof_field = "root_z_dof", .joint = "root_z",
         .qpos_const = f.idx.root_z_qpos, .dof_const = f.idx.root_z_dof, .joint_id = f.ids.root_z},
        {.qpos_field = "root_pitch_qpos", .dof_field = "root_pitch_dof", .joint = "root_pitch",
         .qpos_const = f.idx.root_pitch_qpos, .dof_const = f.idx.root_pitch_dof,
         .joint_id = f.ids.root_pitch},
        {.qpos_field = "crank_spin_qpos", .dof_field = "crank_spin_dof", .joint = "crank_spin",
         .qpos_const = f.idx.crank_spin_qpos, .dof_const = f.idx.crank_spin_dof,
         .joint_id = f.ids.crank_spin},
        {.qpos_field = "fork_travel_qpos", .dof_field = "fork_travel_dof", .joint = "fork_travel",
         .qpos_const = f.idx.fork_travel_qpos, .dof_const = f.idx.fork_travel_dof,
         .joint_id = f.ids.fork_travel},
        {.qpos_field = "front_wheel_spin_qpos", .dof_field = "front_wheel_spin_dof",
         .joint = "front_wheel_spin", .qpos_const = f.idx.front_wheel_spin_qpos,
         .dof_const = f.idx.front_wheel_spin_dof, .joint_id = f.ids.front_wheel_spin},
        {.qpos_field = "rear_wheel_spin_qpos", .dof_field = "rear_wheel_spin_dof",
         .joint = "rear_wheel_spin", .qpos_const = f.idx.rear_wheel_spin_qpos,
         .dof_const = f.idx.rear_wheel_spin_dof, .joint_id = f.ids.rear_wheel_spin},
        {.qpos_field = "shock_stroke_qpos", .dof_field = "shock_stroke_dof", .joint = "shock_stroke",
         .qpos_const = f.idx.shock_stroke_qpos, .dof_const = f.idx.shock_stroke_dof,
         .joint_id = f.ids.shock_stroke},
        {.qpos_field = "drive_shaft_spin_qpos", .dof_field = "drive_shaft_spin_dof",
         .joint = "drive_shaft_spin", .qpos_const = f.idx.drive_shaft_spin_qpos,
         .dof_const = f.idx.drive_shaft_spin_dof, .joint_id = f.ids.drive_shaft_spin},
    }};
    for (const JointEntry &e : joints) {
        const std::string joint = "joint " + std::string(e.joint);
        check_index(table, e.qpos_field, joint, e.qpos_const, f.mv.jnt_qposadr[to_index(e.joint_id)]);
        check_index(table, e.dof_field, joint, e.dof_const, f.mv.jnt_dofadr[to_index(e.joint_id)]);
    }

    check_index(table, "main_pivot_dof", "joint main_pivot", f.idx.main_pivot_dof,
                f.mv.jnt_dofadr[to_index(f.ids.main_pivot)]);
    check_index(table, "horst_pivot_dof", "joint horst_pivot", f.idx.horst_pivot_dof,
                f.mv.jnt_dofadr[to_index(f.ids.horst_pivot)]);
    check_index(table, "freehub_tendon", "tendon ideal_mid_drive_freehub", f.idx.freehub_tendon,
                f.ids.freehub);
    check_index(table, "crank_clutch_tendon", "tendon crank_clutch", f.idx.crank_clutch_tendon,
                f.ids.crank_clutch);

    const auto rider_qpos = std::span{f.idx.rider_qpos};
    const auto rider_dof = std::span{f.idx.rider_dof};
    const auto rider_ctrl = std::span{f.idx.rider_ctrl};
    for (int j = 0; j < kNumRiderJoints; ++j) {
        const std::size_t uj = to_index(j);
        const std::string joint = "joint " + std::string(kRiderJointNames[uj]);
        const int jid = f.ids.rider_joint[uj];
        check_index(table, std::format("rider_qpos[{}]", j), joint, rider_qpos[uj],
                    f.mv.jnt_qposadr[to_index(jid)]);
        check_index(table, std::format("rider_dof[{}]", j), joint, rider_dof[uj],
                    f.mv.jnt_dofadr[to_index(jid)]);
        check_index(table, std::format("rider_ctrl[{}]", j),
                    "actuator act_" + std::string(kRiderJointNames[uj]), rider_ctrl[uj],
                    f.ids.rider_act[uj]);
    }
}

// ---------------------------------------------------------------------------------------------
// Poses. Each pose is applied from qpos0 and evaluated with mj_kinematics + mj_comPos.
// ---------------------------------------------------------------------------------------------

struct Pose {
    std::string_view label;
    double root_x;
    double root_z;
    double root_pitch;   // rad, hinge about +y
    double fork_frac;    // 0 -> fork_travel range lower limit, 1 -> upper limit
    double main_pivot;   // rad, unlimited hinge
    double horst_pivot;  // rad, unlimited hinge
};

constexpr std::array<Pose, 9> kPoses{{
    {.label = "qpos0",
     .root_x = 0.0, .root_z = 0.0, .root_pitch = 0.0, .fork_frac = 0.0,
     .main_pivot = 0.0, .horst_pivot = 0.0},
    {.label = "root_pitch=+0.25",
     .root_x = 0.0, .root_z = 0.0, .root_pitch = 0.25, .fork_frac = 0.5,
     .main_pivot = 0.0, .horst_pivot = 0.0},
    {.label = "root_pitch=-0.25 root=(0.7,-0.1)",
     .root_x = 0.7, .root_z = -0.1, .root_pitch = -0.25, .fork_frac = 0.5,
     .main_pivot = 0.0, .horst_pivot = 0.0},
    {.label = "fork=range_lo pitch=+0.25",
     .root_x = 0.0, .root_z = 0.0, .root_pitch = 0.25, .fork_frac = 0.0,
     .main_pivot = 0.0, .horst_pivot = 0.0},
    {.label = "fork=range_hi",
     .root_x = 0.0, .root_z = 0.0, .root_pitch = 0.0, .fork_frac = 1.0,
     .main_pivot = 0.0, .horst_pivot = 0.0},
    {.label = "main_pivot=+0.05",
     .root_x = 0.0, .root_z = 0.0, .root_pitch = 0.0, .fork_frac = 0.5,
     .main_pivot = 0.05, .horst_pivot = 0.0},
    {.label = "main_pivot=-0.05 pitch=+0.25",
     .root_x = 0.0, .root_z = 0.0, .root_pitch = 0.25, .fork_frac = 0.5,
     .main_pivot = -0.05, .horst_pivot = 0.0},
    {.label = "horst_pivot=+0.05",
     .root_x = 0.0, .root_z = 0.0, .root_pitch = 0.0, .fork_frac = 0.5,
     .main_pivot = 0.0, .horst_pivot = 0.05},
    {.label = "combined root=(0.3,0.2) pitch=+0.25 fork=mid main=-0.05 horst=+0.05",
     .root_x = 0.3, .root_z = 0.2, .root_pitch = 0.25, .fork_frac = 0.5,
     .main_pivot = -0.05, .horst_pivot = 0.05},
}};

void set_qpos(const Fixture &f, int jid, double value, std::string_view name) {
    if (f.mv.jnt_limited[to_index(jid)] != 0) {
        const std::span<const mjtNum> range = f.mv.jnt_range.subspan(2 * to_index(jid), 2);
        const double lo = range[0];
        const double hi = range[1];
        require(value >= lo && value <= hi,
                std::format("pose value {}={:.9g} outside jnt_range [{:.9g}, {:.9g}]",
                            name, value, lo, hi));
    }
    f.dv.qpos[to_index(f.mv.jnt_qposadr[to_index(jid)])] = value;
}

void apply_pose(const Fixture &f, const Pose &p) {
    std::ranges::copy(f.mv.qpos0, f.dv.qpos.begin());
    std::ranges::fill(f.dv.qvel, 0.0);
    set_qpos(f, f.ids.root_x, p.root_x, "root_x");
    set_qpos(f, f.ids.root_z, p.root_z, "root_z");
    set_qpos(f, f.ids.root_pitch, p.root_pitch, "root_pitch");
    require(f.mv.jnt_limited[to_index(f.ids.fork_travel)] != 0, "fork_travel joint is limited");
    const std::span<const mjtNum> fork_range =
        f.mv.jnt_range.subspan(2 * to_index(f.ids.fork_travel), 2);
    const double lo = fork_range[0];
    const double hi = fork_range[1];
    set_qpos(f, f.ids.fork_travel, lo + p.fork_frac * (hi - lo), "fork_travel");
    set_qpos(f, f.ids.main_pivot, p.main_pivot, "main_pivot");
    set_qpos(f, f.ids.horst_pivot, p.horst_pivot, "horst_pivot");
    mj_kinematics(f.m.get(), f.d.get());
    mj_comPos(f.m.get(), f.d.get());
}

float xpos_f(const std::vector<float> &xpos, int body, int comp) {
    return xpos[3 * to_index(body) + to_index(comp)];
}

// Everything a per-pose check needs: the fixture, the pose's float32 xpos
// snapshot and the pose-derived inputs the GPU helpers take.
struct PoseState {
    const Fixture *f;
    const std::vector<float> *xpos;
    float com_x;
    float com_z;
    float pitch;
    float main_pivot;
    std::string ctx;
    CheckTable *table;
};

// ---------------------------------------------------------------------------------------------
// Geometry checks.
// ---------------------------------------------------------------------------------------------

// The projection functions take the root reference point as "com". Production callers pass
// xpos[frame], which is the root_pitch hinge anchor (root_pitch sits at the frame origin). This
// check pins that convention against MuJoCo so a future model edit cannot silently break it.
void check_root_pitch_anchor(const PoseState &s) {
    const std::span<const mjtNum> anchor = row3(s.f->dv.xanchor, s.f->ids.root_pitch);
    const std::span<const mjtNum> frame = row3(s.f->dv.xpos, s.f->ids.frame);
    const double ax = anchor[0];
    const double az = anchor[2];
    const double fx = frame[0];
    const double fz = frame[2];
    s.table->record("root_pitch_anchor==frame_origin",
                   std::max(std::fabs(ax - fx), std::fabs(az - fz)), kAnchorTolM,
                   std::format("{} anchor=({:.9g},{:.9g}) frame=({:.9g},{:.9g})",
                               s.ctx, ax, az, fx, fz));
}

// fork_travel world axis (x, z), normalised, versus gpu_fork_axis_world. The world path derives
// the frame orientation from the steer-to-frame vector; the fallback path (xpos == nullptr)
// uses the kForkAxisLocal constants rotated by root_pitch. Both are checked.
void check_fork_axis(const PoseState &s) {
    const std::span<const mjtNum> axis = row3(s.f->dv.xaxis, s.f->ids.fork_travel);
    const double wx = axis[0];
    const double wz = axis[2];
    const double n = std::hypot(wx, wz);
    const double rx = wx / n;
    const double rz = wz / n;

    const GpuVec2 world = bike::gpu::gpu_fork_axis_world(s.f->idx, s.xpos->data(), s.pitch);
    s.table->record("fork_axis(world path)",
                   std::max(std::fabs(static_cast<double>(world.x) - rx),
                            std::fabs(static_cast<double>(world.y) - rz)),
                   kAxisTol,
                   std::format("{} got=({:.9g},{:.9g}) ref=({:.9g},{:.9g})",
                               s.ctx, world.x, world.y, rx, rz));

    const GpuVec2 fallback = bike::gpu::gpu_fork_axis_world(s.f->idx, nullptr, s.pitch);
    s.table->record("fork_axis(fallback constants)",
                   std::max(std::fabs(static_cast<double>(fallback.x) - rx),
                            std::fabs(static_cast<double>(fallback.y) - rz)),
                   kAxisTol,
                   std::format("{} got=({:.9g},{:.9g}) ref=({:.9g},{:.9g})",
                               s.ctx, fallback.x, fallback.y, rx, rz));
}

// Chainstay (main pivot) anchor versus d->xanchor of the hinge. The world path (body origin from
// xpos) and the fallback (frame-local offset rotated by root_pitch) are both checked at every pose.
// The chainstay origin is frame-local, so the fallback does not depend on main_pivot.
void check_main_pivot_anchor(const PoseState &s) {
    const std::span<const mjtNum> anchor = row3(s.f->dv.xanchor, s.f->ids.main_pivot);
    const double ref_x = anchor[0];
    const double ref_z = anchor[2];

    const GpuVec2 world = bike::gpu::gpu_rear_pivot_anchor_world(
        s.f->idx.chainstay_body, s.xpos->data(), s.com_x, s.com_z,
        bike::gpu::kMainPivotLocalDxM, bike::gpu::kMainPivotLocalDzM, s.pitch);
    s.table->record("main_pivot_anchor(world path)",
                   std::max(std::fabs(static_cast<double>(world.x) - ref_x),
                            std::fabs(static_cast<double>(world.y) - ref_z)),
                   kAnchorTolM,
                   std::format("{} got=({:.9g},{:.9g}) ref=({:.9g},{:.9g})",
                               s.ctx, world.x, world.y, ref_x, ref_z));

    const GpuVec2 fallback = bike::gpu::gpu_rear_pivot_anchor_world(
        -1, nullptr, s.com_x, s.com_z, bike::gpu::kMainPivotLocalDxM,
        bike::gpu::kMainPivotLocalDzM, s.pitch);
    s.table->record("main_pivot_anchor(fallback)",
                   std::max(std::fabs(static_cast<double>(fallback.x) - ref_x),
                            std::fabs(static_cast<double>(fallback.y) - ref_z)),
                   kAnchorTolM,
                   std::format("{} got=({:.9g},{:.9g}) ref=({:.9g},{:.9g})",
                               s.ctx, fallback.x, fallback.y, ref_x, ref_z));
}

// Horst (seatstay) pivot anchor versus d->xanchor of the horst hinge. The world path reads
// xpos[seatstay]. The fallback is rebuilt from root_pitch AND main_pivot, both at the pose's values:
// the seatstay origin sits on the chainstay, so a rotated main_pivot moves it by about 0.0169 m at
// 0.05 rad if ignored.
void check_horst_pivot_anchor(const PoseState &s) {
    const std::span<const mjtNum> anchor = row3(s.f->dv.xanchor, s.f->ids.horst_pivot);
    const double ref_x = anchor[0];
    const double ref_z = anchor[2];

    const GpuVec2 world = bike::gpu::gpu_horst_pivot_anchor_world(
        s.f->idx.seatstay_body, s.xpos->data(), s.com_x, s.com_z, s.pitch, s.main_pivot);
    s.table->record("horst_pivot_anchor(world path)",
                   std::max(std::fabs(static_cast<double>(world.x) - ref_x),
                            std::fabs(static_cast<double>(world.y) - ref_z)),
                   kAnchorTolM,
                   std::format("{} got=({:.9g},{:.9g}) ref=({:.9g},{:.9g})",
                               s.ctx, world.x, world.y, ref_x, ref_z));

    const GpuVec2 fallback =
        bike::gpu::gpu_horst_pivot_anchor_fallback(s.com_x, s.com_z, s.pitch, s.main_pivot);
    s.table->record("horst_pivot_anchor(fallback)",
                   std::max(std::fabs(static_cast<double>(fallback.x) - ref_x),
                            std::fabs(static_cast<double>(fallback.y) - ref_z)),
                   kAnchorTolM,
                   std::format("{} got=({:.9g},{:.9g}) ref=({:.9g},{:.9g})",
                               s.ctx, fallback.x, fallback.y, ref_x, ref_z));
}

struct NamedDof {
    int dof;
    std::string_view name;
};

// The wheel one force projection is applied to: the body the contact point
// rides on, the tire geom (for its radius) and the dofs the projection is
// documented to write. Bundled so the front/rear call sites stay unambiguous.
struct WheelCheck {
    bool front;
    int body;
    int tire_geom;
    const std::vector<NamedDof> *named;
};

// Planar contact force on a wheel versus mj_applyFT. Contact points are on the tire surface at
// several angles (-90 deg is "wheel centre minus radius in world z"). Only the dofs that the
// projection function is documented to write are compared against MuJoCo; every other dof of
// its output must stay exactly zero.
void check_wheel_force(const PoseState &s, const WheelCheck &w, bool use_xpos) {
    const int nv = s.f->counts.nv;
    // use_xpos=false passes xpos=nullptr, so the projection takes its frame-local fallback path.
    const float *xpos_arg = use_xpos ? s.xpos->data() : nullptr;
    const std::string qfrc_check =
        std::string(w.front ? "front_wheel_qfrc" : "rear_wheel_qfrc") +
        (use_xpos ? "" : "(fallback)");
    const std::string unwritten_check = qfrc_check + "(unwritten dofs stay 0)";
    const double radius = row3(s.f->mv.geom_size, w.tire_geom)[0];
    const std::span<const mjtNum> centre = row3(s.f->dv.xpos, w.body);
    const double cx = centre[0];
    const double cy = centre[1];
    const double cz = centre[2];

    const std::array<double, 3> contact_deg = {-90.0, -60.0, 30.0};
    const std::array<std::array<double, 2>, 3> force_dir = {{{1.0, 0.0}, {0.0, -1.0}, {0.37, -0.93}}};
    constexpr double kForceMagN = 500.0;
    constexpr double pi = std::numbers::pi;

    for (const double deg : contact_deg) {
        const double a = deg * pi / 180.0;
        const double px = cx + radius * std::cos(a);
        const double pz = cz + radius * std::sin(a);
        const std::array<mjtNum, 3> point = {px, cy, pz};
        for (const std::array<double, 2> &dir : force_dir) {
            const double fx = kForceMagN * dir[0];
            const double fz = kForceMagN * dir[1];
            const std::array<mjtNum, 3> force = {fx, 0.0, fz};
            const std::array<mjtNum, 3> torque = {0.0, 0.0, 0.0};

            std::vector<mjtNum> ref(to_index(nv), 0.0);
            mj_applyFT(s.f->m.get(), s.f->d.get(), force.data(), torque.data(), point.data(),
                       w.body, ref.data());

            std::vector<float> out(to_index(nv), 0.0f);
            if (w.front) {
                bike::gpu::project_front_wheel_force_to_qfrc(
                    s.f->idx, xpos_arg, s.com_x, s.com_z, static_cast<float>(px),
                    static_cast<float>(pz), static_cast<float>(fx), static_cast<float>(fz),
                    out.data(), s.pitch);
            } else {
                bike::gpu::project_rear_wheel_force_to_qfrc(
                    s.f->idx, xpos_arg, s.com_x, s.com_z, static_cast<float>(px),
                    static_cast<float>(pz), static_cast<float>(fx), static_cast<float>(fz),
                    out.data(), s.pitch, s.main_pivot);
            }

            // Lever-arm-scaled tolerance: relative 1e-4 of |F| * kLeverBoundM, plus a tiny floor.
            const double tol = kQfrcRelTol * std::hypot(fx, fz) * kLeverBoundM + kQfrcAbsFloorNm;
            const std::string ctx =
                std::format("{} contact_deg={:.0f} F=({:.1f},{:.1f})", s.ctx, deg, fx, fz);

            std::vector<bool> is_named(to_index(nv), false);
            for (const NamedDof &nd : *w.named) {
                const std::size_t ud = to_index(nd.dof);
                is_named[ud] = true;
                s.table->record(qfrc_check, std::fabs(static_cast<double>(out[ud]) - ref[ud]), tol,
                                std::format("{} dof={}({}) got={:.9g} ref={:.9g}", ctx, nd.dof,
                                            nd.name, static_cast<double>(out[ud]), ref[ud]));
            }
            for (int dof = 0; dof < nv; ++dof) {
                const std::size_t ud = to_index(dof);
                if (is_named[ud]) {
                    continue;
                }
                s.table->record(unwritten_check, std::fabs(static_cast<double>(out[ud])), 0.0,
                                std::format("{} dof={} got={:.9g} (expected untouched)", ctx, dof,
                                            static_cast<double>(out[ud])));
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Cases. Each case runs its check(s) at all nine poses; the record() order
// inside a case matches the old monolithic run() per pose, and the case order
// matches the old stats-table order.
// ---------------------------------------------------------------------------------------------

void test_indices(const Fixture &f) {
    CheckTable table;
    check_indices(f, table);
    finish_checks(table);
}

// Runs fn on every pose in kPoses, in order, after staging the pose and
// refreshing the float32 xpos snapshot the GPU helpers consume.
template <typename F>
void run_poses(const Fixture &f, CheckTable &table, const F &fn) {
    std::vector<float> xpos(f.dv.xpos.size(), 0.0f);
    for (const Pose &pose : kPoses) {
        apply_pose(f, pose);
        for (std::size_t i = 0; i < xpos.size(); ++i) {
            xpos[i] = static_cast<float>(f.dv.xpos[i]);
        }
        fn(PoseState{
            .f = &f,
            .xpos = &xpos,
            .com_x = xpos_f(xpos, f.ids.frame, 0),
            .com_z = xpos_f(xpos, f.ids.frame, 2),
            .pitch = static_cast<float>(pose.root_pitch),
            .main_pivot = static_cast<float>(pose.main_pivot),
            .ctx = std::format("pose '{}'", pose.label),
            .table = &table,
        });
    }
}

void test_root_pitch_anchor(const Fixture &f) {
    CheckTable table;
    run_poses(f, table, [](const PoseState &s) { check_root_pitch_anchor(s); });
    finish_checks(table);
}

void test_fork_axis(const Fixture &f) {
    CheckTable table;
    run_poses(f, table, [](const PoseState &s) { check_fork_axis(s); });
    finish_checks(table);
}

void test_main_pivot_anchor(const Fixture &f) {
    CheckTable table;
    run_poses(f, table, [](const PoseState &s) { check_main_pivot_anchor(s); });
    finish_checks(table);
}

void test_horst_pivot_anchor(const Fixture &f) {
    CheckTable table;
    run_poses(f, table, [](const PoseState &s) { check_horst_pivot_anchor(s); });
    finish_checks(table);
}

void test_wheel_force_qfrc(const Fixture &f) {
    const std::vector<NamedDof> front_named{
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.root_x)], .name = "root_x"},
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.root_z)], .name = "root_z"},
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.root_pitch)], .name = "root_pitch"},
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.fork_travel)], .name = "fork_travel"},
    };
    const std::vector<NamedDof> rear_named{
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.root_x)], .name = "root_x"},
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.root_z)], .name = "root_z"},
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.root_pitch)], .name = "root_pitch"},
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.main_pivot)], .name = "main_pivot"},
        {.dof = f.mv.jnt_dofadr[to_index(f.ids.horst_pivot)], .name = "horst_pivot"},
    };
    const WheelCheck front{
        .front = true, .body = f.ids.front_wheel, .tire_geom = f.ids.front_tire,
        .named = &front_named,
    };
    const WheelCheck rear{
        .front = false, .body = f.ids.rear_wheel, .tire_geom = f.ids.rear_tire,
        .named = &rear_named,
    };
    CheckTable table;
    run_poses(f, table, [&front, &rear](const PoseState &s) {
        check_wheel_force(s, front, /*use_xpos=*/true);
        check_wheel_force(s, rear, /*use_xpos=*/true);
        check_wheel_force(s, rear, /*use_xpos=*/false);
    });
    finish_checks(table);
}

void test_wheel_chain_and_jacobian(const Fixture &f) {
    using namespace bike::physics_core;

    const GpuWheelChain front_chain = build_wheel_chain(f.m.get(), f.ids.front_wheel);
    const GpuWheelChain rear_chain = build_wheel_chain(f.m.get(), f.ids.rear_wheel);
    require(front_chain.joint_count == 5, "welded.xml front_wheel must have 5-DOF chain");
    require(rear_chain.joint_count == 6, "welded.xml rear_wheel must have 6-DOF chain");
    require(front_chain.njnt == f.counts.njnt && rear_chain.njnt == f.counts.njnt,
            "chain njnt must equal model njnt");

    const std::span<const GpuChainJoint, kMaxChainJoints> front_joints(front_chain.joints);
    const std::array<int, 5> expected_front_jids = {
        f.ids.root_x, f.ids.root_z, f.ids.root_pitch, f.ids.fork_travel, f.ids.front_wheel_spin};
    for (std::size_t k = 0; k < expected_front_jids.size(); ++k) {
        require(front_joints[k].joint_id == expected_front_jids[k],
                "front_wheel chain joint order mismatch");
    }

    const std::span<const GpuChainJoint, kMaxChainJoints> rear_joints(rear_chain.joints);
    const std::array<int, 6> expected_rear_jids = {
        f.ids.root_x, f.ids.root_z, f.ids.root_pitch, f.ids.main_pivot, f.ids.horst_pivot,
        f.ids.rear_wheel_spin};
    for (std::size_t k = 0; k < expected_rear_jids.size(); ++k) {
        require(rear_joints[k].joint_id == expected_rear_jids[k],
                "rear_wheel chain joint order mismatch");
    }

    const int nv = f.counts.nv;
    const std::size_t jac_size = 3 * to_index(nv);
    std::vector<double> ref_jp(jac_size, 0.0);
    std::vector<double> ref_jr(jac_size, 0.0);
    std::vector<double> got_jp_no_jr(jac_size, 0.0);
    std::vector<double> got_jp(jac_size, 0.0);
    std::vector<double> got_jr(jac_size, 0.0);
    std::vector<float> got_jp_f(jac_size, 0.0f);
    std::vector<float> got_jr_f(jac_size, 0.0f);

    std::vector<float> xpos_f32(f.dv.xpos.size(), 0.0f);
    std::vector<float> xanchor_f32(f.dv.xanchor.size(), 0.0f);
    std::vector<float> xaxis_f32(f.dv.xaxis.size(), 0.0f);

    double max_err_f64 = 0.0;
    double max_err_f32 = 0.0;

    for (const Pose &pose : kPoses) {
        apply_pose(f, pose);
        for (std::size_t i = 0; i < xpos_f32.size(); ++i) {
            xpos_f32[i] = static_cast<float>(f.dv.xpos[i]);
        }
        for (std::size_t i = 0; i < xanchor_f32.size(); ++i) {
            xanchor_f32[i] = static_cast<float>(f.dv.xanchor[i]);
            xaxis_f32[i] = static_cast<float>(f.dv.xaxis[i]);
        }

        const std::array<std::pair<const GpuWheelChain *, int>, 2> wheels{{
            {&front_chain, f.ids.front_wheel},
            {&rear_chain, f.ids.rear_wheel},
        }};
        for (const auto &[chain_ptr, body_id] : wheels) {
            const std::span<const mjtNum> center = row3(f.dv.xpos, body_id);
            const std::array<std::array<double, 3>, 3> points{{
                {center[0], center[1], center[2]},
                {center[0] + 0.12, center[1], center[2] - 0.35},
                {center[0] - 0.20, center[1] + 0.04, center[2] - 0.31},
            }};
            for (const auto &pt : points) {
                std::ranges::fill(ref_jp, 0.0);
                std::ranges::fill(ref_jr, 0.0);
                mj_jac(f.m.get(), f.d.get(), ref_jp.data(), ref_jr.data(), pt.data(), body_id);

                ErrCode err = ErrCode::none;
                const Status st_no_jr = chain_point_jacobian<double, CpuExactToolkit, false>(
                    *chain_ptr, f.dv.xpos.data(), f.dv.xanchor.data(), f.dv.xaxis.data(),
                    body_id, nv, pt.data(), got_jp_no_jr.data(), nullptr, err);
                require(st_no_jr == Status::ok && err == ErrCode::none,
                        "chain_point_jacobian<double, false> must succeed");

                const Status st_jr = chain_point_jacobian<double, CpuExactToolkit, true>(
                    *chain_ptr, f.dv.xpos.data(), f.dv.xanchor.data(), f.dv.xaxis.data(),
                    body_id, nv, pt.data(), got_jp.data(), got_jr.data(), err);
                require(st_jr == Status::ok && err == ErrCode::none,
                        "chain_point_jacobian<double, true> must succeed");

                const std::array<float, 3> pt_f = {
                    static_cast<float>(pt[0]), static_cast<float>(pt[1]), static_cast<float>(pt[2])};
                const Status st_f32 = chain_point_jacobian<float, GpuToolkit, true>(
                    *chain_ptr, xpos_f32.data(), xanchor_f32.data(), xaxis_f32.data(),
                    body_id, nv, pt_f.data(), got_jp_f.data(), got_jr_f.data(), err);
                require(st_f32 == Status::ok && err == ErrCode::none,
                        "chain_point_jacobian<float, true> must succeed");

                for (std::size_t idx = 0; idx < jac_size; ++idx) {
                    require(got_jp_no_jr[idx] == got_jp[idx],
                            "ComputeJr=false and ComputeJr=true jp must match identically");
                    max_err_f64 = std::max(max_err_f64, std::abs(got_jp[idx] - ref_jp[idx]));
                    max_err_f64 = std::max(max_err_f64, std::abs(got_jr[idx] - ref_jr[idx]));
                    max_err_f32 = std::max(
                        max_err_f32, std::abs(static_cast<double>(got_jp_f[idx]) - ref_jp[idx]));
                    max_err_f32 = std::max(
                        max_err_f32, std::abs(static_cast<double>(got_jr_f[idx]) - ref_jr[idx]));
                }
            }
        }
    }
    require(max_err_f64 <= 1e-12,
            std::format("FP64 chain_point_jacobian max_err {:.3e} > 1e-12", max_err_f64));
    require(max_err_f32 <= 1e-5,
            std::format("FP32 chain_point_jacobian max_err {:.3e} > 1e-5", max_err_f32));

    // Bounds rejections in chain_point_jacobian
    const std::array<double, 3> dummy_pt = {0.0, 0.0, 0.0};
    ErrCode err = ErrCode::none;
    {
        GpuWheelChain bad = front_chain;
        bad.joint_count = kMaxChainJoints + 1;
        require(chain_point_jacobian<double, CpuExactToolkit, false>(
                    bad, f.dv.xpos.data(), f.dv.xanchor.data(), f.dv.xaxis.data(),
                    f.ids.front_wheel, nv, dummy_pt.data(), got_jp.data(), nullptr, err) ==
                    Status::out_of_bounds &&
                err == ErrCode::out_of_bounds_chain,
                "chain_point_jacobian rejects joint_count > kMaxChainJoints");
    }
    {
        GpuWheelChain bad = front_chain;
        std::span<GpuChainJoint, kMaxChainJoints>(bad.joints)[0].joint_id = f.counts.njnt;
        require(chain_point_jacobian<double, CpuExactToolkit, false>(
                    bad, f.dv.xpos.data(), f.dv.xanchor.data(), f.dv.xaxis.data(),
                    f.ids.front_wheel, nv, dummy_pt.data(), got_jp.data(), nullptr, err) ==
                    Status::out_of_bounds &&
                err == ErrCode::out_of_bounds_chain,
                "chain_point_jacobian rejects joint_id >= njnt");
    }
    {
        GpuWheelChain bad = front_chain;
        std::span<GpuChainJoint, kMaxChainJoints>(bad.joints)[0].dof_adr = nv;
        require(chain_point_jacobian<double, CpuExactToolkit, false>(
                    bad, f.dv.xpos.data(), f.dv.xanchor.data(), f.dv.xaxis.data(),
                    f.ids.front_wheel, nv, dummy_pt.data(), got_jp.data(), nullptr, err) ==
                    Status::out_of_bounds &&
                err == ErrCode::out_of_bounds_chain,
                "chain_point_jacobian rejects dof_adr >= nv");
    }
}

nativecfg::TireConfig make_valid_compliant_cfg() {
    nativecfg::TireMaterial mat{};
    mat.radial_k_n_m = 110000.0;
    mat.loss_factor = 0.05;
    mat.hysteresis_rate_eps_mps = 0.01;
    mat.pressure_pa_gauge = 150000.0;
    mat.provenance = "test";
    mat.valid_load_range_n = {0.0, 4000.0};

    nativecfg::TireParams side{};
    side.material = mat;
    side.tangent_k_n_m = 60000.0;
    side.mu = 1.1;
    side.relaxation_length_m = 0.05;

    nativecfg::TireConfig cfg{};
    cfg.backend = "compliant_2d";
    cfg.surface_mode = "configured";
    cfg.front = side;
    cfg.rear = std::move(side);
    cfg.significant_delta_m = 0.002;
    cfg.significance_fraction = 0.25;
    cfg.distinct_normal_deg = 12.0;
    return cfg;
}

template <class Fn>
void require_invalid_arg(
    const Fn &fn,
    std::string_view label,
    const char *substr = ""
) {
    try {
        fn();
    } catch (const std::invalid_argument &e) {
        const std::string_view expected(substr);
        if (!expected.empty() && std::string_view(e.what()).find(expected) == std::string_view::npos) {
            throw std::runtime_error(
                std::string(label) + ": wrong invalid_argument message: " + e.what());
        }
        return;
    }
    throw std::runtime_error(std::string(label) + ": expected std::invalid_argument");
}

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
[[gnu::noinline]] std::unique_ptr<bike::physics_core::HostTyreUploadBundle>
build_gpu_tyre_model_heap(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    int front_geom,
    int rear_geom
) {
    return std::make_unique<bike::physics_core::HostTyreUploadBundle>(
        bike::physics_core::build_gpu_tyre_model(m, d, cfg, front_geom, rear_geom));
}

[[gnu::noinline]] std::unique_ptr<bike::physics_core::HostTyreUploadBundle>
build_gpu_tyre_model_heap_bodies(
    const mjModel *m,
    const mjData *d,
    const nativecfg::TireConfig &cfg,
    int front_geom,
    int rear_geom,
    int front_body,
    int rear_body
) {
    return std::make_unique<bike::physics_core::HostTyreUploadBundle>(
        bike::physics_core::build_gpu_tyre_model(
            m, d, cfg, front_geom, rear_geom, front_body, rear_body));
}

[[gnu::noinline]] std::unique_ptr<bike::physics_core::HostTyreUploadBundle>
copy_tyre_bundle_heap(const bike::physics_core::HostTyreUploadBundle &src) {
    return std::make_unique<bike::physics_core::HostTyreUploadBundle>(src);
}
NATIVE_DIAG_POP

void test_tyre_upload_compliant(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    const nativecfg::TireConfig compliant_cfg = make_valid_compliant_cfg();
    const auto bundle = build_gpu_tyre_model_heap(
        f.m.get(), f.d.get(), compliant_cfg, f.ids.front_contact, f.ids.rear_contact);
    require(bundle->gpu_model.nq == 27 && bundle->gpu_model.nv == 27 &&
                bundle->gpu_model.njnt == 27,
            "uploaded GpuTyreModel dimensions must match welded.xml nq=27");
    const std::span<const TyreWheelSpec<float>, 2> wheels(bundle->gpu_model.wheels);
    require(wheels[0].chain.joint_count == 5 && wheels[1].chain.joint_count == 6,
            "uploaded GpuTyreModel wheel chains must have 5 and 6 joints");
    require(bundle->polyline_view.num_vertices >= 2 &&
                bundle->signed_profile_view.num_vertices >= 2,
            "uploaded terrain views must be populated");
    require(bundle->polyline_view.px == bundle->pl_px.data(),
            "HostTyreUploadBundle polyline_view.px must point into owned pl_px");
    const auto copied = copy_tyre_bundle_heap(*bundle);
    require(copied->polyline_view.px == copied->pl_px.data() &&
                copied->polyline_view.px != bundle->pl_px.data(),
            "HostTyreUploadBundle copy must rebind view pointers");
}

void test_tyre_upload_distributed(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    nativecfg::TireConfig dist_cfg = make_valid_compliant_cfg();
    dist_cfg.backend = "distributed_2d_reference";
    nativecfg::HingeDensity hd{};
    hd.knots_m = {0.0, 0.02};
    hd.stiffness_n_m = {50000.0, 80000.0};
    hd.loss_factor = 0.05;
    hd.hysteresis_rate_eps_mps = 0.01;
    hd.provenance = "test";
    nativecfg::DistributedMaterial dm{};
    dm.density = hd;
    dm.valid_load_range_n = {0.0, 4000.0};
    dm.valid_deflection_range_m = {0.0, 0.1};
    dist_cfg.distributed = nativecfg::DistributedConfig{
        .station_count = 64,
        .density = hd,
        .front = dm,
        .rear = dm,
    };
    const auto dist_bundle = build_gpu_tyre_model_heap(
        f.m.get(), f.d.get(), dist_cfg, f.ids.front_contact, f.ids.rear_contact);
    require(dist_bundle->gpu_model.station_count == 64, "distributed station_count == 64");
}

[[gnu::noinline]] void test_tyre_upload_tabulated(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    nativecfg::TireConfig tab_cfg = make_valid_compliant_cfg();
    require(tab_cfg.front.material.has_value(), "compliant cfg must have front material");
    if (tab_cfg.front.material.has_value()) {
        tab_cfg.front.material->table = nativecfg::TireTable{
            .deflection_m = {0.0, 0.02, 0.05},
            .force_n = {0.0, 1800.0, 5000.0},
        };
    }
    const auto bundle = build_gpu_tyre_model_heap(
        f.m.get(), f.d.get(), tab_cfg, f.ids.front_contact, f.ids.rear_contact);
    require(bundle->gpu_model.backend == TyreBackendKind::compliant_2d,
            "TabulatedRadial upload keeps compliant_2d backend");
    const std::span<const TyreWheelSpec<float>, 2> wheels(bundle->gpu_model.wheels);
    const CompliantMaterialView<float> &fc = wheels[0].compliant;
    const CompliantMaterialView<float> &rc = wheels[1].compliant;
    require(fc.has_tabulated_radial == 1 && fc.use_tabulated == 1 && fc.knot_count == 3,
            "front wheel must enable TabulatedRadial with knot_count == 3");
    require(rc.has_tabulated_radial == 0 && rc.use_tabulated == 0,
            "rear wheel without table must keep use_tabulated == 0");
    const std::span<const float, kMaxGpuRadialTableKnots> f_defl(fc.deflection_m);
    const std::span<const float, kMaxGpuRadialTableKnots> f_force(fc.force_n);
    const std::span<const float, kMaxGpuRadialTableKnots - 1> f_slopes(fc.slopes);
    const std::span<const float, kMaxGpuRadialTableKnots> f_integ(fc.integral);
    require(std::abs(f_defl[1] - 0.02f) <= 1e-6f && std::abs(f_defl[2] - 0.05f) <= 1e-6f,
            "TabulatedRadial deflection knots populated");
    require(std::abs(f_force[1] - 1800.0f) <= 1e-3f && std::abs(f_force[2] - 5000.0f) <= 1e-3f,
            "TabulatedRadial force_n knots populated");
    require(std::abs(f_slopes[0] - 90000.0f) <= 1.0f && f_slopes[1] > 100000.0f,
            "TabulatedRadial slopes populated");
    require(f_integ[0] == 0.0f && f_integ[1] > 10.0f && f_integ[2] > f_integ[1],
            "TabulatedRadial integral populated monotonically");
}

[[gnu::noinline]] void test_tyre_upload_pneumatic(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    const ModelPtr m_copy(mj_copyModel(nullptr, f.m.get()), &mj_deleteModel);
    const std::span<mjtNum> gsize =
        model_span(m_copy->geom_size, 3 * static_cast<std::size_t>(m_copy->ngeom));
    gsize[3 * to_index(f.ids.front_contact) + 0] = 0.379;
    gsize[3 * to_index(f.ids.rear_contact) + 0] = 0.360;

    const auto make_pneu_cfg = [](bool detailed) {
        nativecfg::TireConfig cfg = make_valid_compliant_cfg();
        cfg.backend = "pneumatic";
        cfg.surface_mode = "track";
        cfg.significant_delta_m = 0.0001;
        cfg.significance_fraction = 0.05;
        cfg.distinct_normal_deg = 20.0;

        nativecfg::PneumaticTyreSpec front{};
        front.name = "Schwalbe Magic Mary 29x2.8";
        front.outer_radius_mm = 379.0;
        front.rim_radius_mm = 320.0;
        front.section_height_mm = 68.0;
        front.pressure_bar = 1.2;
        front.crr_reference = 0.011;
        front.casing_width_mm = 65.7;
        front.compressed_casing_mm = 6.0;
        front.area_factor = 0.509;
        front.carcass_stiffness_n_mm2 = 0.136;
        front.contact_length_factor = 0.851;
        front.rate_stiffening = 0.22;
        front.rate_relaxation_s = 0.2;
        front.loss_factor = 0.055;
        front.hysteresis_rate_eps_mps = 0.01;
        front.relaxation_length_mm = 60.0;
        front.rim_stiffness_n_mm2 = 28.0;
        front.tread_loss_crr = 0.00586;

        nativecfg::PneumaticTyreSpec rear = front;
        rear.name = "Schwalbe Hans Dampf 27.5x2.8";
        rear.outer_radius_mm = 360.0;
        rear.rim_radius_mm = 300.0;
        rear.crr_reference = 0.0103;
        rear.area_factor = 0.522;
        rear.carcass_stiffness_n_mm2 = 0.140;
        rear.contact_length_factor = 0.827;
        rear.tread_loss_crr = 0.00481;

        cfg.pneumatic = nativecfg::PneumaticConfig{
            .tier = detailed ? "detailed" : "fast",
            .n_rays = detailed ? 256 : 64,
            .half_angle_deg = 75.0,
            .discretised_brush = detailed,
            .timestep_s = detailed ? 0.00025 : 0.0005,
            .surface = std::nullopt,
            .front = std::move(front),
            .rear = std::move(rear),
            .profile = {},
        };

        nativecfg::SurfaceSpec hardpack{
            .name = "hardpack",
            .mu_peak = 0.80,
            .mu_slide = 0.60,
            .slip_stiffness_per_load = 12.0,
            .stribeck_speed_mps = 4.5,
            .crr_delta = 0.0025,
            .load_sensitivity = 0.08,
        };
        nativecfg::SurfaceSpec scree{
            .name = "scree",
            .mu_peak = 0.45,
            .mu_slide = 0.38,
            .slip_stiffness_per_load = 4.5,
            .stribeck_speed_mps = 4.5,
            .crr_delta = 0.0080,
            .load_sensitivity = 0.15,
        };
        cfg.surface_map = nativecfg::SurfaceMap{
            .surface = std::move(hardpack),
            .sections = {
                nativecfg::SurfaceSection{
                    .start_m = 4.0,
                    .end_m = 12.0,
                    .surface = std::move(scree),
                    .blend_m = 1.5,
                },
            },
        };
        return cfg;
    };

    for (const bool detailed : {false, true}) {
        const nativecfg::TireConfig pneu_cfg = make_pneu_cfg(detailed);
        const auto bundle = build_gpu_tyre_model_heap(
            m_copy.get(), f.d.get(), pneu_cfg, f.ids.front_contact, f.ids.rear_contact);
        require(bundle->gpu_model.backend == TyreBackendKind::pneumatic,
                "pneumatic upload sets TyreBackendKind::pneumatic");
        const i32 expected_rays = detailed ? 256 : 64;
        const u8 expected_brush = detailed ? 1 : 0;
        const std::span<const TyreWheelSpec<float>, 2> wheels(bundle->gpu_model.wheels);
        for (std::size_t s = 0; s < 2; ++s) {
            require(wheels[s].pneumatic.n_rays == expected_rays,
                    "pneumatic uploaded n_rays must match tier");
            require(wheels[s].pneumatic.discretised_brush == expected_brush,
                    "pneumatic uploaded discretised_brush must match tier");
            require(wheels[s].pneumatic.dtheta_rad > 0.0f,
                    "pneumatic ray ring dtheta_rad populated");
        }
        const SurfaceMapView<float> &smap = bundle->gpu_model.surface_map;
        require(smap.has_surface_map == 1 && smap.section_count == 1,
                "Stage A surface_map uploaded for pneumatic");
        require(std::abs(smap.default_surface.crr_delta - 0.0025f) <= 1e-6f &&
                    std::abs(smap.default_surface.load_sensitivity - 0.08f) <= 1e-6f,
                "Stage A default surface crr_delta and load_sensitivity populated");
        const std::span<const SurfaceSectionView<float>, kMaxSurfaceSections> secs(smap.sections);
        require(std::abs(secs[0].blend_m - 1.5f) <= 1e-6f &&
                    std::abs(secs[0].surface.crr_delta - 0.0080f) <= 1e-6f &&
                    std::abs(secs[0].surface.load_sensitivity - 0.15f) <= 1e-6f,
                "Stage A section blend_m, crr_delta, and load_sensitivity populated");
    }
}

[[gnu::noinline]] void test_tyre_upload_reject_invalid_inputs(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    {
        nativecfg::TireConfig bad_tab = make_valid_compliant_cfg();
        require(bad_tab.front.material.has_value(), "compliant cfg must have front material");
        if (bad_tab.front.material.has_value()) {
            bad_tab.front.material->table = nativecfg::TireTable{
                .deflection_m = {0.0, 0.03, 0.02},
                .force_n = {0.0, 1500.0, 2000.0},
            };
        }
        require_invalid_arg(
            [&]() {
                (void)build_gpu_tyre_model_heap(
                    f.m.get(), f.d.get(), bad_tab, f.ids.front_contact, f.ids.rear_contact);
            },
            "reject non-monotonic TabulatedRadial knots", "TabulatedRadial");
    }
    {
        nativecfg::TireConfig bad_backend = make_valid_compliant_cfg();
        bad_backend.backend = "unknown_tyre_backend";
        require_invalid_arg(
            [&]() {
                (void)build_gpu_tyre_model_heap(
                    f.m.get(), f.d.get(), bad_backend, f.ids.front_contact, f.ids.rear_contact);
            },
            "reject unknown tyre backend", "unknown_tyre_backend");
    }
    {
        nativecfg::TireConfig bad_pneu = make_valid_compliant_cfg();
        bad_pneu.backend = "pneumatic";
        require_invalid_arg(
            [&]() {
                (void)build_gpu_tyre_model_heap(
                    f.m.get(), f.d.get(), bad_pneu, f.ids.front_contact, f.ids.rear_contact);
            },
            "reject pneumatic without pneumatic config");
    }
    {
        nativecfg::SurfaceSpec hardpack{
            .name = "hardpack",
            .mu_peak = 0.80,
            .mu_slide = 0.60,
            .slip_stiffness_per_load = 12.0,
            .stribeck_speed_mps = 4.5,
            .crr_delta = 0.0025,
            .load_sensitivity = 0.08,
        };
        nativecfg::SurfaceSpec wet_wood{
            .name = "wet_wood",
            .mu_peak = 0.25,
            .mu_slide = 0.18,
            .slip_stiffness_per_load = 9.0,
            .stribeck_speed_mps = 4.5,
            .crr_delta = 0.0030,
            .load_sensitivity = 0.12,
        };
        nativecfg::TireConfig bad_micro = make_valid_compliant_cfg();
        bad_micro.surface_mode = "track";
        bad_micro.surface_map = nativecfg::SurfaceMap{
            .surface = hardpack,
            .sections = {},
            .micro_spans = {
                nativecfg::MicroSurfaceSpan{
                    .start_m = 4.0,
                    .end_m = 5.0,
                    .blend_m = 0.1,
                    .surface = wet_wood,
                },
            },
        };
        require_invalid_arg(
            [&]() {
                (void)build_gpu_tyre_model_heap(
                    f.m.get(), f.d.get(), bad_micro, f.ids.front_contact, f.ids.rear_contact);
            },
            "reject obstacle micro_spans in GPU tyre upload", "micro_spans");

        nativecfg::SurfaceSpec loose = hardpack;
        loose.name = "loose";
        nativecfg::TireConfig bad_layer = make_valid_compliant_cfg();
        bad_layer.surface_mode = "track";
        bad_layer.surface_map = nativecfg::SurfaceMap{
            .surface = hardpack,
            .sections = {
                nativecfg::SurfaceSection{
                    .start_m = 4.0,
                    .end_m = 12.0,
                    .surface = loose,
                    .blend_m = 1.0,
                    .layer_thickness_m = 0.035,
                },
            },
        };
        require_invalid_arg(
            [&]() {
                (void)build_gpu_tyre_model_heap(
                    f.m.get(), f.d.get(), bad_layer, f.ids.front_contact, f.ids.rear_contact);
            },
            "reject deformable layer_thickness_m in GPU tyre upload", "layer_thickness_m");
    }
}

void test_tyre_upload_reject_station_count(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    nativecfg::TireConfig bad_dist = make_valid_compliant_cfg();
    bad_dist.backend = "distributed_2d_reference";
    nativecfg::HingeDensity hd{};
    hd.knots_m = {0.0, 0.02};
    hd.stiffness_n_m = {50000.0, 80000.0};
    hd.loss_factor = 0.05;
    hd.hysteresis_rate_eps_mps = 0.01;
    hd.provenance = "test";
    nativecfg::DistributedMaterial dm{};
    dm.density = hd;
    dm.valid_load_range_n = {0.0, 4000.0};
    dm.valid_deflection_range_m = {0.0, 0.1};
    bad_dist.distributed = nativecfg::DistributedConfig{
        .station_count = 129,
        .density = hd,
        .front = dm,
        .rear = dm,
    };
    require_invalid_arg(
        [&]() {
            (void)build_gpu_tyre_model_heap(
                f.m.get(), f.d.get(), bad_dist, f.ids.front_contact, f.ids.rear_contact);
        },
        "reject station_count > 128");
}

void test_tyre_upload_reject_body_mismatch(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    const nativecfg::TireConfig compliant_cfg = make_valid_compliant_cfg();
    require_invalid_arg(
        [&]() {
            (void)build_gpu_tyre_model_heap_bodies(
                f.m.get(), f.d.get(), compliant_cfg, f.ids.front_contact, f.ids.rear_contact,
                f.ids.rear_wheel, f.ids.rear_wheel);
        },
        "reject geom_bodyid != wheel_body");
}

void test_tyre_upload_reject_geom_pos(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    const nativecfg::TireConfig compliant_cfg = make_valid_compliant_cfg();
    const ModelPtr m_copy(mj_copyModel(nullptr, f.m.get()), &mj_deleteModel);
    const std::span<mjtNum> gpos =
        model_span(m_copy->geom_pos, 3 * static_cast<std::size_t>(m_copy->ngeom));
    gpos[3 * to_index(f.ids.front_contact) + 0] = 1e-4;
    require_invalid_arg(
        [&]() {
            (void)build_gpu_tyre_model_heap(
                m_copy.get(), f.d.get(), compliant_cfg, f.ids.front_contact, f.ids.rear_contact);
        },
        "reject non-zero geom_pos");
}

void test_tyre_upload_reject_geom_quat(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    const nativecfg::TireConfig compliant_cfg = make_valid_compliant_cfg();
    const ModelPtr m_copy(mj_copyModel(nullptr, f.m.get()), &mj_deleteModel);
    const std::span<mjtNum> gquat =
        model_span(m_copy->geom_quat, 4 * static_cast<std::size_t>(m_copy->ngeom));
    gquat[4 * to_index(f.ids.front_contact) + 0] = 0.0;
    gquat[4 * to_index(f.ids.front_contact) + 1] = 1.0;
    require_invalid_arg(
        [&]() {
            (void)build_gpu_tyre_model_heap(
                m_copy.get(), f.d.get(), compliant_cfg, f.ids.front_contact, f.ids.rear_contact);
        },
        "reject non-identity geom_quat");
}

void test_tyre_upload_reject_bad_chain(const Fixture &f) {
    using namespace bike::physics_core;

    apply_pose(f, kPoses[0]);
    const nativecfg::TireConfig compliant_cfg = make_valid_compliant_cfg();
    {
        const ModelPtr m_copy(mj_copyModel(nullptr, f.m.get()), &mj_deleteModel);
        const std::span<int> bjadr =
            model_span(m_copy->body_jntadr, static_cast<std::size_t>(m_copy->nbody));
        bjadr[to_index(f.ids.front_wheel)] = static_cast<int>(m_copy->njnt);
        require_invalid_arg(
            [&]() { (void)build_wheel_chain(m_copy.get(), f.ids.front_wheel); },
            "reject joint_id >= njnt in build_wheel_chain");
        require_invalid_arg(
            [&]() {
                (void)build_gpu_tyre_model_heap(
                    m_copy.get(), f.d.get(), compliant_cfg, f.ids.front_contact, f.ids.rear_contact);
            },
            "reject joint_id >= njnt in build_gpu_tyre_model");
    }
    {
        const ModelPtr m_copy(mj_copyModel(nullptr, f.m.get()), &mj_deleteModel);
        const std::span<int> jtype =
            model_span(m_copy->jnt_type, static_cast<std::size_t>(m_copy->njnt));
        jtype[0] = mjJNT_FREE;
        require_invalid_arg(
            [&]() { (void)build_wheel_chain(m_copy.get(), f.ids.front_wheel); },
            "reject mjJNT_FREE in build_wheel_chain");
    }
}

std::filesystem::path resolve_model_xml(std::string_view arg) {
    namespace fs = std::filesystem;
    if (!arg.empty()) {
        return {arg};
    }
    std::vector<fs::path> candidates;
#ifdef BIKE_REPO_ROOT
    candidates.emplace_back(fs::path(BIKE_REPO_ROOT) / kRelModelPath);
#endif
    candidates.emplace_back(kRelModelPath);
    candidates.emplace_back(fs::path("..") / kRelModelPath);
    candidates.emplace_back(fs::path("../..") / kRelModelPath);
    for (const fs::path &p : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(p, ec)) {
            return fs::canonical(p, ec);
        }
    }
    return {};
}

constexpr std::array<TestCase, 17> cases{{
    {.name = "indices", .run = test_indices},
    {.name = "root_pitch_anchor", .run = test_root_pitch_anchor},
    {.name = "fork_axis", .run = test_fork_axis},
    {.name = "main_pivot_anchor", .run = test_main_pivot_anchor},
    {.name = "horst_pivot_anchor", .run = test_horst_pivot_anchor},
    {.name = "wheel_force_qfrc", .run = test_wheel_force_qfrc},
    {.name = "wheel_chain_and_jacobian", .run = test_wheel_chain_and_jacobian},
    {.name = "tyre_upload_compliant", .run = test_tyre_upload_compliant},
    {.name = "tyre_upload_distributed", .run = test_tyre_upload_distributed},
    {.name = "tyre_upload_tabulated", .run = test_tyre_upload_tabulated},
    {.name = "tyre_upload_pneumatic", .run = test_tyre_upload_pneumatic},
    {.name = "tyre_upload_reject_invalid_inputs", .run = test_tyre_upload_reject_invalid_inputs},
    {.name = "tyre_upload_reject_station_count", .run = test_tyre_upload_reject_station_count},
    {.name = "tyre_upload_reject_body_mismatch", .run = test_tyre_upload_reject_body_mismatch},
    {.name = "tyre_upload_reject_geom_pos", .run = test_tyre_upload_reject_geom_pos},
    {.name = "tyre_upload_reject_geom_quat", .run = test_tyre_upload_reject_geom_quat},
    {.name = "tyre_upload_reject_bad_chain", .run = test_tyre_upload_reject_bad_chain},
}};

int run_case(const TestCase &test_case, const Fixture &fixture) {
    try {
        test_case.run(fixture);
        std::cout << "PASS " << test_case.name << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
        return 1;
    }
}
} // namespace

int main(int argc, char **argv) {
    try {
        const std::span<char *> arguments = bike::platform::main_arguments(argc, argv);
        // argv[1..] classification: an argument ending in ".xml" is the model
        // path, anything else is a case name. Both may appear in either order;
        // "--list" must be the only argument.
        std::string_view model_arg;
        std::string_view requested;
        bool case_seen = false;
        for (std::size_t i = 1; i < arguments.size(); ++i) {
            const std::string_view arg(arguments[i]);
            if (arg == "--list") {
                if (arguments.size() == 2) {
                    for (const TestCase &test_case : cases) {
                        std::cout << test_case.name << '\n';
                    }
                    return 0;
                }
                std::cerr << "usage: bike_test_gpu_model_geometry [model.xml] [case-name|--list]\n";
                return 2;
            }
            if (arg.ends_with(".xml")) {
                if (!model_arg.empty()) {
                    std::cerr
                        << "usage: bike_test_gpu_model_geometry [model.xml] [case-name|--list]\n";
                    return 2;
                }
                model_arg = arg;
            } else {
                if (case_seen) {
                    std::cerr
                        << "usage: bike_test_gpu_model_geometry [model.xml] [case-name|--list]\n";
                    return 2;
                }
                case_seen = true;
                requested = arg;
            }
        }
        if (!case_seen) {
            requested = "all";
        }

        // An unknown case name is a usage error (exit 2) before any model I/O.
        bool found = requested == "all";
        for (const TestCase &test_case : cases) {
            found = found || requested == test_case.name;
        }
        if (!found) {
            std::cerr << "unknown gpu model_geometry case: " << requested << '\n';
            return 2;
        }

        const std::filesystem::path xml = resolve_model_xml(model_arg);
        if (xml.empty()) {
            std::cerr << "FAIL: " << kRelModelPath
                      << " not found (pass the path as an .xml argument or run from the repo "
                         "root)\n";
            return 1;
        }

        const Fixture fixture = make_fixture(xml);
        std::cout << "model: " << xml.string() << '\n';
        int status = 0;
        for (const TestCase &test_case : cases) {
            if (requested == "all" || requested == test_case.name) {
                if (run_case(test_case, fixture) != 0) {
                    status = 1;
                }
            }
        }
        return status;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
