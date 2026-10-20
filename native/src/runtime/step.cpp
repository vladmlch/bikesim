// runtime/step.cpp — the owned physical step (plan A3).
//
// Ports PhysicalRuntime.apply_forces/_advance_physics statement-for-
// statement. Every forward/write site mirrors its named Python line —
// the order is the floating-point contract. Ordered force maps stay
// insertion-ordered pair vectors (Python dict order is the wire
// contract). Everything below is GIL-free: metadata leaves as Wire
// trees the binding boxes after the mutable section commits.

#include "step.hpp"

#include <algorithm>
#include <cstdint>
#include <bit>
#include <cmath>
#include <ranges>
#include <stdexcept>

#include "../cblas_abi.hpp"
#include "../drivetrain/pedaling.hpp"
#include "../engine_call.hpp"
#include "../numeric_sum.hpp"
#include "../rider/rider_posture.hpp"
#include "../rider/response_codec.hpp"
#include "value_readers.hpp"
#include "../stepper.hpp"
#include "../writers/cruise.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/resistance.hpp"
#include "../writers/rider_forces.hpp"
#include "../writers/suspension.hpp"

namespace runtime {
namespace {

// ---- numeric helpers --------------------------------------------------
// `a @ b` lowers to BLAS ddot — the same entry point numpy dispatches to
// on this platform (cblas_abi.hpp documents the bitwise contract).
// `sum()` over floats is CPython 3.13's Neumaier loop; `np.sum`/`ndarray.sum`
// uses numpy's pairwise block order, ported verbatim below. Every
// NeumaierSum site in this TU feeds Python floats only (patch fields,
// float products, wire_scalars), so CPython's compensated float branch
// is the whole path — no PyLong-addend naive steps are reachable. This
// local accumulator updates `c` unconditionally like both CPython
// versions do (the finite-guard variant lives in numeric_sum.hpp).

struct NeumaierSum {
    double sum = 0., c = 0.;
    void add(double v) noexcept {
        const double t = sum + v;
        c += std::abs(sum) >= std::abs(v) ? (sum - t) + v : (v - t) + sum;
        sum = t;
    }
    [[nodiscard]] double total() const noexcept { return sum + c; }
};

// Engine-buffer span adapters — the same views::counted style the
// Stepper accessors use; raw ptr+size indexing is a hard error here.
std::span<const double> buf(const double *p, mjtSize n) {
    return std::views::counted(p, n);
}
std::span<double> wbuf(double *p, mjtSize n) {
    return std::views::counted(p, n);
}
std::span<const int> ibuf(const int *p, mjtSize n) {
    return std::views::counted(p, n);
}

// np.sum — shared with the mirrored sites outside this TU;
// numeric::numpy_sum documents the exact upstream order (the
// add-reduce 0.0 identity over pairwise_sum, so an all-(-0.0) input
// folds to +0.0 exactly like np.sum).
double numpy_sum(std::span<const double> a) noexcept {
    return numeric::numpy_sum(a);
}

double dot(std::span<const double> a, std::span<const double> b) noexcept {
    return blas::ddot(static_cast<int>(a.size()), a.data(), 1, b.data(),
                      1);
}

double scalar_field(double value, const char *name) {
    if (!std::isfinite(value))
        throw std::invalid_argument(std::string(name) + " must be finite");
    return value;
}

// ---- ordered force map (dict insertion order is the wire contract) ---
using ForceMap = std::vector<std::pair<std::string, std::vector<double>>>;

std::vector<double> *ordered_at(ForceMap &map, std::string_view name) {
    for (auto &[key, value]: map)
        if (key == name)
            return &value;
    return nullptr;
}

const std::vector<double> *ordered_at(const ForceMap &map,
                                      std::string_view name) {
    for (auto &[key, value]: map)
        if (key == name)
            return &value;
    return nullptr;
}

void ordered_set(ForceMap &map, std::string name, std::vector<double> v) {
    if (auto *slot = ordered_at(map, name)) {
        *slot = std::move(v);
        return;
    }
    map.emplace_back(std::move(name), std::move(v));
}

std::vector<double> zeros(const mjModel *model) {
    std::vector<double> out(
        static_cast<std::vector<double>::size_type>(model->nv), 0.);
    return out;
}

// Owned copy of an engine buffer — views::counted keeps clang's
// -Wunsafe-buffer-usage-in-container off the raw two-iterator forms.
std::vector<double> buffer_copy(const double *ptr, mjtSize n) {
    const auto view = std::views::counted(ptr, n);
    return {view.begin(), view.end()};
}

// runtime.address(name) — the joint name -> (qpos, dof) lookup with the
// oracle's ValueError on a missing joint.
int joint_qposadr(const mjModel *m, const char *name) {
    const int jid = mj_name2id(m, mjOBJ_JOINT, name);
    if (jid < 0)
        throw std::invalid_argument(std::string("physical model needs joint '") +
                                    name + "'");
    return ibuf(m->jnt_qposadr, m->njnt)[static_cast<std::size_t>(jid)];
}
int joint_dofadr(const mjModel *m, const char *name) {
    const int jid = mj_name2id(m, mjOBJ_JOINT, name);
    if (jid < 0)
        throw std::invalid_argument(std::string("physical model needs joint '") +
                                    name + "'");
    return ibuf(m->jnt_dofadr, m->njnt)[static_cast<std::size_t>(jid)];
}

// d.sensor(name).data -> sensordata[sensor_adr[id]...]: the sensordata
// row offset is NOT the sensor id (jointpos/jointvel channels above the
// IMU are 1-dim, so id!=adr once any wider channel precedes it). The
// oracle's sensor() lookup raises on a missing name — reject -1 alike.
int sensor_adr(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_SENSOR, name);
    if (id < 0)
        throw std::invalid_argument(std::string("physical model needs sensor '") +
                                    name + "'");
    return ibuf(m->sensor_adr, m->nsensor)[static_cast<std::size_t>(id)];
}

// physical_mapping.py point_jacobian — (jp; jr) row-major (3, nv) blocks,
// one mj_jac call filling both like the Python helper.
struct PointJacobian {
    std::vector<double> jp, jr;
};
PointJacobian point_jacobian(const mjModel *model, const mjData *data,
                             int body,
                             const std::span<const double, 3> point) {
    const std::size_t nv = static_cast<std::size_t>(model->nv);
    PointJacobian out{.jp = std::vector<double>(3 * nv, 0.),
                      .jr = std::vector<double>(3 * nv, 0.)};
    mj_jac(model, data, out.jp.data(), out.jr.data(), point.data(),
           body);
    return out;
}

// ---- WheelContactSnapshot properties (contact_state.py) --------------
// The native TirePatch stores the same primitives; every aggregate below
// mirrors its named Python @property in accumulation order.

double patch_tangent_component(const TirePatch &p, int axis) {
    // tangent = [normal[2], 0, -normal[0]]
    if (axis == 0)
        return p.normal[2];
    if (axis == 2)
        return -p.normal[0];
    return 0.;
}

std::array<double, 3> patch_world_force(const TirePatch &p) {
    if (p.native_world_force_n) return *p.native_world_force_n;
    return {p.normal_load_n * p.normal[0] + p.tangent_force_n * p.normal[2],
            0.,
            p.normal_load_n * p.normal[2] - p.tangent_force_n * p.normal[0]};
}

// sum(generator, 0.0) — CPython sum with a float seed: sequential
// Neumaier additions starting at 0.
double sum_f64(std::initializer_list<double>) = delete;

double snapshot_normal_load(const TireSnapshot &s) {
    NeumaierSum acc;
    for (const auto &p: s.patches)
        acc.add(p.normal_load_n);
    return acc.total();
}
std::array<double, 3> snapshot_world_force(const TireSnapshot &s) {
    std::array<double, 3> out{0., 0., 0.};
    for (const auto &p: s.patches) {
        const auto f = patch_world_force(p);
        for (std::size_t i = 0; i < 3; ++i)
            out[i] += f[i];
    }
    return out;
}
double snapshot_normal_vertical(const TireSnapshot &s) {
    NeumaierSum acc;
    for (const auto &p: s.patches)
        acc.add(p.normal_load_n * p.normal[2]);
    return acc.total();
}
double snapshot_tangent_force(const TireSnapshot &s) {
    NeumaierSum acc;
    for (const auto &p: s.patches)
        acc.add(p.tangent_force_n);
    return acc.total();
}
double snapshot_axis_moment(const TireSnapshot &s) {
    NeumaierSum acc;
    for (const auto &p: s.patches) {
        const auto f = patch_world_force(p);
        const double rx = p.point_m[0] - s.wheel_axis_m[0];
        const double rz = p.point_m[2] - s.wheel_axis_m[2];
        // Applied tire writers retain the original arithmetic. Engine
        // contacts additionally carry their solved torsional couple.
        const double moment = rz * f[0] - rx * f[2];
        acc.add(p.native_world_force_n || p.couple_world_nm[1] != 0.
            ? moment + p.couple_world_nm[1] : moment);
    }
    return acc.total();
}
bool snapshot_road_loaded(const TireSnapshot &s) {
    for (const auto &p: s.patches)
        if (p.working_surface && p.normal_load_n > 0.)
            return true;
    return false;
}
double snapshot_slip(const TireSnapshot &s) {
    if (s.patches.empty())
        return 0.;
    NeumaierSum num, den;
    bool any = false;
    for (const auto &p: s.patches)
        if (p.normal_load_n > 0.)
            any = true;
    for (const auto &p: s.patches) {
        const double w = any ? p.normal_load_n : 1.;
        num.add(p.slip_mps * w);
        den.add(w);
    }
    return num.total() / den.total();
}

// ---- Wire emission ----------------------------------------------------

Wire wire_opt(const std::optional<double> &v) {
    return v.has_value() ? Wire(*v) : Wire(nullptr);
}

Wire wire_vec3(const std::array<double, 3> &v) {
    return wire_array(std::span<const double>(v));
}

// control.py RiderPosture/RideControl asdict field order.
WireObject wire_posture(const rider::RiderPosture &p) {
    WireObject out;
    out.emplace_back("torso_lean_rad", Wire(p.torso_lean_rad));
    out.emplace_back("pelvis_pitch_rad", Wire(p.pelvis_pitch_rad));
    if (p.pelvis_offset_m.has_value())
        out.emplace_back("pelvis_offset_m",
                         wire_array(std::views::counted(
                             p.pelvis_offset_m->data(), 2)));
    else
        out.emplace_back("pelvis_offset_m", Wire(nullptr));
    out.emplace_back("use_saddle", Wire(p.use_saddle));
    return out;
}

WireObject wire_control(const RideControl &c) {
    WireObject out;
    out.emplace_back("motor_torque_nm", wire_opt(c.motor_torque_nm));
    out.emplace_back("motor_limit_nm", wire_opt(c.motor_limit_nm));
    out.emplace_back("human_torque_nm", wire_opt(c.human_torque_nm));
    out.emplace_back("crank_target_rate_rad_s",
                     wire_opt(c.crank_target_rate_rad_s));
    if (c.posture.has_value())
        out.emplace_back("posture", Wire(wire_posture(*c.posture)));
    else
        out.emplace_back("posture", Wire(nullptr));
    out.emplace_back("rider_enabled", Wire(c.rider_enabled));
    return out;
}

// seated_climb.py SeatedClimbIntent — asdict {'posture','effort_ceiling_nm'}.
WireObject wire_intent(const rider::Intent &intent) {
    WireObject out;
    out.emplace_back("posture", Wire(wire_posture(intent.posture)));
    out.emplace_back("effort_ceiling_nm", Wire(intent.effort_ceiling_nm));
    return out;
}

// rider_contacts.py diagnostics dicts — the typed diagnostics structs
// carry every field; emission picks the welded/spring layout per entry.
WireObject wire_patch_diag(const writers::RiderContactPatchDiag &p) {
    WireObject out;
    out.emplace_back("in_platform", Wire(p.in_platform));
    out.emplace_back("normal_load_n", Wire(p.normal_load_n));
    out.emplace_back("gap_m", Wire(p.gap_m));
    out.emplace_back("point_m", wire_vec3(p.point_m));
    out.emplace_back("force_on_rider_n", wire_vec3(p.force_on_rider_n));
    out.emplace_back("radial_energy_j", Wire(p.radial_energy_j));
    out.emplace_back("shear_energy_j", Wire(p.shear_energy_j));
    return out;
}

WireObject wire_support_diag(const writers::RiderContactSupportDiag &s) {
    WireObject out;
    out.emplace_back("enabled", Wire(s.enabled));
    out.emplace_back("in_platform", Wire(s.in_platform));
    out.emplace_back("normal_load_n", Wire(s.normal_load_n));
    if (s.welded) {
        out.emplace_back("would_separate", Wire(s.would_separate));
        out.emplace_back("would_slip", Wire(s.would_slip));
        out.emplace_back("tangent_n", Wire(s.tangent_n));
    }
    out.emplace_back("gap_m", Wire(s.gap_m));
    out.emplace_back("vertical_force_on_rider_n",
                     Wire(s.vertical_force_on_rider_n));
    if (s.detailed) {
        out.emplace_back("tangent_force_n",
                         s.welded ? wire_vec3(s.tangent_force_vec)
                                  : Wire(s.tangent_force_scalar));
        WireArray patches;
        for (const auto &p: s.patches)
            patches.emplace_back(wire_patch_diag(p));
        out.emplace_back("patches", Wire(std::move(patches)));
        out.emplace_back("force_on_rider_n", wire_vec3(s.force_on_rider_n));
        out.emplace_back("force_on_bike_n", wire_vec3(s.force_on_bike_n));
        out.emplace_back("moment_about_rider_origin_nm",
                         wire_vec3(s.moment_nm));
        out.emplace_back("radial_energy_j", Wire(s.radial_energy_j));
        out.emplace_back("shear_energy_j", Wire(s.shear_energy_j));
        out.emplace_back("relative_power_w", Wire(s.relative_power_w));
    }
    return out;
}

// Welded (connect-equality) layout: base keys then the extended block
// when detailed. Spring layout emits the full set unconditionally with
// 'hand_gap_m' ordered mid-dict exactly like the oracle's literal.
template <typename G>
WireObject wire_grip_side(const G &g, bool spring) {
    WireObject out;
    out.emplace_back("enabled", Wire(g.enabled));
    out.emplace_back("reachable", Wire(g.reachable));
    if (!spring) {
        out.emplace_back("hand_gap_m", Wire(g.hand_gap_m));
        if (!g.extended)
            return out;
    }
    out.emplace_back("overloaded", Wire(g.overloaded));
    out.emplace_back("trial_pair_force_n", Wire(g.trial_pair_force_n));
    out.emplace_back("pair_force_limit_n", wire_opt(g.pair_force_limit_n));
    out.emplace_back("release_loss_j", Wire(g.release_loss_j));
    out.emplace_back("shoulder_distance_m", Wire(g.shoulder_distance_m));
    out.emplace_back("arm_reach_m", Wire(g.arm_reach_m));
    if (spring)
        out.emplace_back("hand_gap_m", Wire(g.hand_gap_m));
    out.emplace_back("point_m", wire_vec3(g.point_m));
    out.emplace_back("force_on_rider_n", wire_vec3(g.force_on_rider_n));
    out.emplace_back("force_on_bike_n", wire_vec3(g.force_on_bike_n));
    out.emplace_back("elastic_energy_j", Wire(g.elastic_energy_j));
    return out;
}

// The aggregate 'grip' dict never carries point_m.
WireObject wire_grip_aggregate(const writers::RiderContactGripDiag &g) {
    WireObject out;
    out.emplace_back("enabled", Wire(g.enabled));
    out.emplace_back("reachable", Wire(g.reachable));
    if (!g.welded) {
        out.emplace_back("overloaded", Wire(g.overloaded));
        out.emplace_back("trial_pair_force_n", Wire(g.trial_pair_force_n));
        out.emplace_back("pair_force_limit_n",
                         wire_opt(g.pair_force_limit_n));
        out.emplace_back("release_loss_j", Wire(g.release_loss_j));
        out.emplace_back("shoulder_distance_m",
                         Wire(g.shoulder_distance_m));
        out.emplace_back("arm_reach_m", Wire(g.arm_reach_m));
        out.emplace_back("hand_gap_m", Wire(g.hand_gap_m));
        out.emplace_back("force_on_rider_n", wire_vec3(g.force_on_rider_n));
        out.emplace_back("force_on_bike_n", wire_vec3(g.force_on_bike_n));
        out.emplace_back("elastic_energy_j", Wire(g.elastic_energy_j));
        return out;
    }
    out.emplace_back("hand_gap_m", Wire(g.hand_gap_m));
    if (!g.extended)
        return out;
    out.emplace_back("overloaded", Wire(g.overloaded));
    out.emplace_back("trial_pair_force_n", Wire(g.trial_pair_force_n));
    out.emplace_back("pair_force_limit_n", wire_opt(g.pair_force_limit_n));
    out.emplace_back("release_loss_j", Wire(g.release_loss_j));
    out.emplace_back("shoulder_distance_m", Wire(g.shoulder_distance_m));
    out.emplace_back("arm_reach_m", Wire(g.arm_reach_m));
    out.emplace_back("force_on_rider_n", wire_vec3(g.force_on_rider_n));
    out.emplace_back("force_on_bike_n", wire_vec3(g.force_on_bike_n));
    out.emplace_back("elastic_energy_j", Wire(g.elastic_energy_j));
    return out;
}

// self.diagnostics — saddle, front_pedal, rear_pedal, then the grip set.
WireObject wire_contacts_diagnostics(
    const writers::RiderContactsDiagnostics &diag) {
    WireObject out;
    if (!diag.evaluated)
        return out;
    // Fixed emission shape: every support, then every grip side, then the
    // 'grip' aggregate — reserve the exact key count.
    out.reserve(writers::kSupportCount + writers::kSideCount + 1);
    for (std::size_t i = 0; i < writers::kSupportCount; ++i)
        out.emplace_back(std::string(writers::kSupportNames[i]),
                         Wire(wire_support_diag(diag.supports[i])));
    for (std::size_t i = 0; i < writers::kSideCount; ++i)
        out.emplace_back(std::string("grip_") +
                             std::string(writers::kSideNames[i]),
                         Wire(wire_grip_side(diag.grip_sides[i],
                                             !diag.grip_sides[i].welded)));
    out.emplace_back("grip", Wire(wire_grip_aggregate(diag.grip)));
    return out;
}

// settle_welds' result dict — support entries carry the five-key set;
// 'grip_*' entries carry force_on_rider_n only; 'crank_torque_nm' is the
// conditional tail appended after every entry.
WireObject wire_settled_entry(const writers::RiderSettledEntry &e) {
    WireObject out;
    out.emplace_back("force_on_rider_n", wire_vec3(e.force_on_rider_n));
    if (e.support_fields) {
        out.emplace_back("normal_n", Wire(e.normal_n));
        out.emplace_back("tangent_n", Wire(e.tangent_n));
        out.emplace_back("would_separate", Wire(e.would_separate));
        out.emplace_back("would_slip", Wire(e.would_slip));
    }
    return out;
}

// tire_forces.py per-side diagnostics dict — insertion order fixed.
WireObject wire_tire_diagnostics(const TireDiagnostics &d) {
    WireObject out;
    out.emplace_back("multi_support", Wire(d.multi_support));
    out.emplace_back("penetration_m", Wire(d.penetration_m));
    out.emplace_back("normal_speed_mps", Wire(d.normal_speed_mps));
    out.emplace_back("slip_mps", Wire(d.slip_mps));
    out.emplace_back("normal_load_n", Wire(d.normal_load_n));
    out.emplace_back("tangent_force_n", Wire(d.tangent_force_n));
    out.emplace_back("friction_coefficient", Wire(d.friction_coefficient));
    out.emplace_back("surface", Wire(d.surface));
    out.emplace_back("branch_release_loss_j", Wire(d.branch_release_loss_j));
    out.emplace_back("brush_loss_j", Wire(d.brush_loss_j));
    out.emplace_back("radial_energy_j", Wire(d.radial_energy_j));
    out.emplace_back("shear_energy_j", Wire(d.shear_energy_j));
    out.emplace_back("outside_material_load_range",
                     Wire(d.outside_material_load_range));
    return out;
}

// distributed_tire_forces.py:204-213 — the distributed per-side dict,
// its own key set in the oracle's insertion order.
WireObject wire_distributed_tire_diagnostics(const TireDiagnostics &d) {
    WireObject out;
    out.emplace_back("penetration_m", Wire(d.penetration_m));
    out.emplace_back("normal_load_n", Wire(d.normal_load_n));
    out.emplace_back("multi_support", Wire(d.multi_support));
    out.emplace_back("supports_multiple_contacts",
                     Wire(d.supports_multiple_contacts));
    out.emplace_back("capability_status", Wire(d.capability_status));
    out.emplace_back("calibration_status", Wire(d.calibration_status));
    out.emplace_back("station_count", Wire(d.station_count));
    out.emplace_back("active_station_count", Wire(d.active_station_count));
    out.emplace_back("nonsmooth_station_count",
                     Wire(d.nonsmooth_station_count));
    out.emplace_back("max_patch_count", Wire(d.max_patch_count));
    out.emplace_back("outside_material_load_range",
                     Wire(d.outside_material_load_range));
    out.emplace_back("outside_material_deflection_range",
                     Wire(d.outside_material_deflection_range));
    out.emplace_back("radial_energy_j", Wire(d.radial_energy_j));
    out.emplace_back("shear_energy_j", Wire(d.shear_energy_j));
    out.emplace_back("brush_loss_j", Wire(d.brush_loss_j));
    return out;
}

// pneumatic_tire_forces.py:426-455 — the pneumatic per-side dict, its
// own key set in the oracle's insertion order.
WireObject wire_pneumatic_tire_diagnostics(const TireDiagnostics &d) {
    WireObject out;
    out.emplace_back("normal_load_n", Wire(d.normal_load_n));
    out.emplace_back("support_n", Wire(d.support_n));
    out.emplace_back("tangent_force_n", Wire(d.tangent_force_n));
    out.emplace_back("slip_mps", Wire(d.slip_mps));
    out.emplace_back("slip_ratio", Wire(d.slip_ratio));
    out.emplace_back("transient_slip_ratio",
                     Wire(d.transient_slip_ratio));
    out.emplace_back("friction_coefficient", Wire(d.friction_coefficient));
    out.emplace_back("penetration_m", Wire(d.penetration_m));
    out.emplace_back("contact_length_m", Wire(d.contact_length_m));
    out.emplace_back("patch_count", Wire(d.patch_count));
    out.emplace_back("multi_support", Wire(d.multi_support));
    out.emplace_back("supports_multiple_contacts",
                     Wire(d.supports_multiple_contacts));
    out.emplace_back("fully_sliding", Wire(d.fully_sliding));
    out.emplace_back("airborne", Wire(d.airborne));
    out.emplace_back("coverage_event", Wire(d.coverage_event));
    out.emplace_back("rim_strike_active", Wire(d.rim_strike_active));
    out.emplace_back("rim_event_x_m", Wire(d.rim_event_x_m));
    out.emplace_back("rim_event_speed_mps", Wire(d.rim_event_speed_mps));
    out.emplace_back("rim_event_peak_load_n",
                     Wire(d.rim_event_peak_load_n));
    out.emplace_back("rim_event_peak_force_n",
                     Wire(d.rim_event_peak_force_n));
    out.emplace_back("rim_event_energy_j", Wire(d.rim_event_energy_j));
    out.emplace_back("pressure_bar", Wire(d.pressure_bar));
    out.emplace_back("dissipated_power_w", Wire(d.dissipated_power_w));
    out.emplace_back("brush_loss_j", Wire(d.brush_loss_j));
    out.emplace_back("radial_energy_j", Wire(d.radial_energy_j));
    out.emplace_back("shear_energy_j", Wire(d.shear_energy_j));
    out.emplace_back("outside_material_load_range",
                     Wire(d.outside_material_load_range));
    out.emplace_back("surface", Wire(d.surface));
    return out;
}

// ---- observation helpers (physical_observations.py) -------------------

// actuator_components — rows keyed by the bare actuator name in id
// order (the model names its motors 'act_<joint>'; the Python dict keys
// them verbatim — physical_observations.py:113).
ForceMap actuator_components(const mjModel *model, const mjData *data) {
    const auto trnid = ibuf(model->actuator_trnid, 2 * model->nu);
    const auto trntype = ibuf(model->actuator_trntype, model->nu);
    const auto dofadr = ibuf(model->jnt_dofadr, model->njnt);
    const auto force = buf(data->actuator_force, model->nu);
    const auto gear = buf(model->actuator_gear, 6 * model->nu);
    ForceMap out;
    for (int aid = 0; aid < model->nu; ++aid) {
        const auto uaid = static_cast<std::size_t>(aid);
        const int jid = trnid[uaid * 2];
        if (trntype[uaid] != mjTRN_JOINT)
            throw std::invalid_argument(
                "physical actuation requires direct joint motors");
        auto row = zeros(model);
        row[static_cast<std::size_t>(
            dofadr[static_cast<std::size_t>(jid)])] =
            force[uaid] * gear[uaid * 6];
        const char *name = mj_id2name(model, mjOBJ_ACTUATOR, aid);
        out.emplace_back(name != nullptr ? name : "", std::move(row));
    }
    return out;
}

// constraint_components — one mulJacTVec per row group; group insertion
// order is fixed: native_contact, joint_limits, closure.
ForceMap constraint_components(const mjModel *model, const mjData *data,
                               std::vector<double> &weights) {
    const int nefc = data->nefc;
    const auto efc_type = ibuf(data->efc_type, nefc);
    const auto efc_force = buf(data->efc_force, nefc);
    std::array<std::vector<int>, 3> groups;
    for (int i = 0; i < nefc; ++i)
        switch (efc_type[static_cast<std::size_t>(i)]) {
            case mjCNSTR_CONTACT_FRICTIONLESS:
            case mjCNSTR_CONTACT_PYRAMIDAL:
            case mjCNSTR_CONTACT_ELLIPTIC:
                groups[0].push_back(i);
                break;
            case mjCNSTR_LIMIT_JOINT:
            case mjCNSTR_LIMIT_TENDON:
                groups[1].push_back(i);
                break;
            case mjCNSTR_EQUALITY:
                groups[2].push_back(i);
                break;
            default: break;
        }
    static constexpr std::array<std::string_view, 3> names = {
        "native_contact", "joint_limits", "closure"};
    ForceMap out;
    for (std::size_t g = 0; g < 3; ++g) {
        auto force = zeros(model);
        if (!groups[g].empty()) {
            std::ranges::fill(weights, 0.);
            for (const int row: groups[g])
                weights[static_cast<std::size_t>(row)] =
                    efc_force[static_cast<std::size_t>(row)];
            engine::mul_jac_t_vec(model, data, force.data(), weights.data());
        }
        out.emplace_back(std::string(names[g]), std::move(force));
    }
    return out;
}

// shock_joint_limit_qfrc — shock_stroke LIMIT_JOINT rows only.
std::vector<double> shock_joint_limit_qfrc(const mjModel *model,
                                           const mjData *data,
                                           int shock_joint,
                                           std::vector<double> &weights) {
    auto qfrc = zeros(model);
    if (data->nefc == 0)
        return qfrc;
    const auto efc_type = ibuf(data->efc_type, data->nefc);
    const auto efc_id = ibuf(data->efc_id, data->nefc);
    const auto efc_force = buf(data->efc_force, data->nefc);
    bool any = false;
    std::ranges::fill(weights, 0.);
    for (int i = 0; i < data->nefc; ++i)
        if (efc_type[static_cast<std::size_t>(i)] == mjCNSTR_LIMIT_JOINT &&
            efc_id[static_cast<std::size_t>(i)] == shock_joint) {
            weights[static_cast<std::size_t>(i)] =
                efc_force[static_cast<std::size_t>(i)];
            any = true;
        }
    if (any)
        engine::mul_jac_t_vec(model, data, qfrc.data(), weights.data());
    return qfrc;
}

// numerical_constraint_powers — per '{kind}:{efc_id}' accumulated
// force*(J@v) in row order; dict key order is first-appearance order.
std::vector<std::pair<std::string, double>>
numerical_constraint_powers(const mjModel *model, const mjData *data,
                            std::span<const double> incoming,
                            std::vector<double> &velocity) {
    const int nefc = data->nefc;
    velocity.assign(static_cast<std::size_t>(nefc), 0.);
    if (nefc > 0)
        engine::mul_jac_vec(model, data, velocity.data(), incoming.data());
    const auto efc_type = ibuf(data->efc_type, nefc);
    const auto efc_id = ibuf(data->efc_id, nefc);
    const auto efc_force = buf(data->efc_force, nefc);
    std::vector<std::pair<std::string, double>> powers;
    for (int i = 0; i < nefc; ++i) {
        const char *kind = nullptr;
        switch (efc_type[static_cast<std::size_t>(i)]) {
            case mjCNSTR_EQUALITY: kind = "equality"; break;
            case mjCNSTR_LIMIT_JOINT: kind = "joint_limit"; break;
            case mjCNSTR_LIMIT_TENDON: kind = "tendon_limit"; break;
            default: break;
        }
        if (kind == nullptr)
            continue;
        const std::string key =
            std::string(kind) + ":" +
            std::to_string(efc_id[static_cast<std::size_t>(i)]);
        const double power =
            efc_force[static_cast<std::size_t>(i)] *
            velocity[static_cast<std::size_t>(i)];
        bool found = false;
        for (auto &[k, v]: powers)
            if (k == key) {
                v += power;
                found = true;
                break;
            }
        if (!found)
            powers.emplace_back(key, power);
    }
    return powers;
}

// _connect_equality_rows — equality rows owned by connect equalities that
// are NOT rider attachments (connect_saddle/grip_/foot_ stay inside
// rider_contacts' weld accounting).
std::vector<int> connect_equality_rows(const mjModel *model,
                                       const mjData *data) {
    const auto efc_type = ibuf(data->efc_type, data->nefc);
    const auto efc_id = ibuf(data->efc_id, data->nefc);
    const auto eq_type = ibuf(model->eq_type, model->neq);
    std::vector<int> rows;
    for (int i = 0; i < data->nefc; ++i) {
        if (efc_type[static_cast<std::size_t>(i)] != mjCNSTR_EQUALITY)
            continue;
        const int eq = efc_id[static_cast<std::size_t>(i)];
        if (eq_type[static_cast<std::size_t>(eq)] != mjEQ_CONNECT)
            continue;
        const char *name = mj_id2name(model, mjOBJ_EQUALITY, eq);
        const std::string_view n = name != nullptr ? name : "";
        if (n.starts_with("connect_saddle") ||
            n.starts_with("connect_grip_") || n.starts_with("connect_foot_"))
            continue;
        rows.push_back(i);
    }
    return rows;
}

// engine_passive_loss_power — damping/fluid loss after refunding the
// explicit joint springs (their energy is a stored term, not a loss).
double engine_passive_loss_power(const mjModel *model,
                                 // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) (qpos, qvel, passive) order mirrors the Python helper
                                 std::span<const double> qpos,
                                 std::span<const double> qvel,
                                 std::span<const double> passive) {
    std::vector<double> force(passive.begin(), passive.end());
    const auto gravcomp = buf(model->body_gravcomp, model->nbody);
    for (int j = 0; j < model->nbody; ++j)
        if (gravcomp[static_cast<std::size_t>(j)] != 0.)
            throw std::invalid_argument(
                "gravity compensation is not a passive material loss");
    const auto jtype = ibuf(model->jnt_type, model->njnt);
    const auto jstiff = buf(model->jnt_stiffness, model->njnt);
    const auto qposadr = ibuf(model->jnt_qposadr, model->njnt);
    const auto dofadr = ibuf(model->jnt_dofadr, model->njnt);
    const auto spring = buf(model->qpos_spring, model->nq);
    for (int j = 0; j < model->njnt; ++j) {
        const std::size_t u = static_cast<std::size_t>(j);
        if (jstiff[u] == 0.)
            continue;
        if (jtype[u] != mjJNT_HINGE && jtype[u] != mjJNT_SLIDE)
            throw std::invalid_argument(
                "spring accounting requires scalar planar joints");
        const int qa = qposadr[u], va = dofadr[u];
        force[static_cast<std::size_t>(va)] +=
            jstiff[u] *
            (qpos[static_cast<std::size_t>(qa)] -
             spring[static_cast<std::size_t>(qa)]);
    }
    const double power = -dot(force, qvel);
    if (power < -1e-8)
        throw std::overflow_error(
            "unclassified active force in native passive channel");
    return std::max(0., power);
}

// _contacts' per-side `loaded` — working-surface patches only
// (physical_runtime.py:181).
double working_load(const TireSnapshot &s) {
    NeumaierSum acc;
    for (const auto &p: s.patches)
        if (p.working_surface)
            acc.add(p.normal_load_n);
    return acc.total();
}

// components[name] — the oracle's direct index; a missing row is a
// hard contract break (KeyError -> std::out_of_range).
const std::vector<double> &require_component(const ForceMap &map,
                                             std::string_view name) {
    if (const auto *row = ordered_at(map, name))
        return *row;
    throw std::out_of_range("missing force component '" +
                            std::string(name) + "'");
}

// components[name] -= ... — the mutable face for the oracle's in-place
// row arithmetic (dict[key] -= vector on an always-present key).
std::vector<double> &require_component_mut(ForceMap &map,
                                           std::string_view name) {
    if (auto *row = ordered_at(map, name))
        return *row;
    throw std::out_of_range("missing force component '" +
                            std::string(name) + "'");
}

// ---- Wire object readers (signals_from_channels / dict.get shapes) -----

const Wire *wire_at(const WireObject &o, std::string_view key) {
    for (const auto &[k, v]: o)
        if (k == key)
            return &v;
    return nullptr;
}

double wire_scalar(const Wire &w, double fallback = 0.) noexcept {
    if (const auto *x = std::get_if<double>(&w.value))
        return *x;
    if (const auto *x = std::get_if<std::int64_t>(&w.value))
        return static_cast<double>(*x);
    if (const auto *x = std::get_if<bool>(&w.value))
        return *x ? 1. : 0.;
    return fallback;
}

double wire_num(const WireObject &o, std::string_view key,
                double fallback = 0.) noexcept {
    const Wire *w = wire_at(o, key);
    return w != nullptr ? wire_scalar(*w, fallback) : fallback;
}

const WireObject *wire_obj(const WireObject &o, std::string_view key) {
    const Wire *w = wire_at(o, key);
    if (w == nullptr)
        return nullptr;
    return std::get_if<WireObject>(&w->value);
}

const WireArray *wire_arr(const WireObject &o, std::string_view key) {
    const Wire *w = wire_at(o, key);
    if (w == nullptr)
        return nullptr;
    return std::get_if<WireArray>(&w->value);
}

double wire_arr_at(const WireObject &o, std::string_view key,
                   std::size_t index) noexcept {
    const WireArray *a = wire_arr(o, key);
    if (a == nullptr || index >= a->size())
        return 0.;
    return wire_scalar((*a)[index]);
}

// dict.update / dict(base, **kw) semantics — replace in place when the key
// exists (the insertion position is kept), else append.
void wire_set(WireObject &o, std::string_view key, Wire value) {
    for (auto &[k, v]: o)
        if (k == key) {
            v = std::move(value);
            return;
        }
    o.emplace_back(std::string(key), std::move(value));
}

// drivetrain::DiagnosticValue -> Wire (monostate is Python None).
Wire diag_wire(const drivetrain::DiagnosticValue &v) {
    if (const auto *x = std::get_if<double>(&v))
        return Wire{*x};
    if (const auto *x = std::get_if<int>(&v))
        return Wire{static_cast<std::int64_t>(*x)};
    if (const auto *x = std::get_if<bool>(&v))
        return Wire{*x};
    if (const auto *x = std::get_if<std::string>(&v))
        return Wire{*x};
    return Wire{nullptr};
}

// drive.last.get(key, 0.) — absent yields the float default; a present
// None stays None (the wire dict's own rule).
Wire diag_wire_default_zero(const drivetrain::Diagnostics &m,
                            const std::string &key) {
    const auto it = m.find(key);
    return it == m.end() ? Wire(0.) : diag_wire(it->second);
}

// drive.last.get(key, 0.) under float() — the sensor-channel readers and
// the loss ledger's `*dt` terms; non-numeric/absent is 0.
double diag_number(const drivetrain::Diagnostics &m, const char *key) {
    const auto it = m.find(key);
    if (it == m.end())
        return 0.;
    if (const auto *x = std::get_if<double>(&it->second))
        return *x;
    if (const auto *x = std::get_if<int>(&it->second))
        return static_cast<double>(*x);
    if (const auto *x = std::get_if<bool>(&it->second))
        return *x ? 1. : 0.;
    return 0.;
}

// ---- rider wire dicts (spindle_wire.cpp emission orders, GIL-free) -----

WireObject wire_named_reals(const rider::NamedEntries<double> &entries) {
    WireObject out;
    out.reserve(entries.size());
    for (const auto &[name, value]: entries)
        out.emplace_back(name, Wire(value));
    return out;
}

WireObject wire_named_bools(const rider::NamedEntries<bool> &entries) {
    WireObject out;
    out.reserve(entries.size());
    for (const auto &[name, value]: entries)
        out.emplace_back(name, Wire(value));
    return out;
}

WireArray wire_names(const std::vector<std::string> &names) {
    WireArray out;
    out.reserve(names.size());
    for (const auto &name: names)
        out.emplace_back(name);
    return out;
}

// allocation_diagnostics_dict — the four keys in emission order, only
// when present (Python emits {} for a not-run controller).
WireObject
wire_allocation(const rider::AllocationDiagnostics &d) {
    if (!d.present)
        return {};
    if (d.legacy) {
        auto const value = bike::rider::encode_response_allocation(*d.legacy);
        auto wire = vread::wire_from_value(value, "channels.rider_allocation");
        return std::get<WireObject>(std::move(wire.value));
    }
    return {{"invalid_controller", Wire(d.invalid_controller)},
            {"crank_task_nm", Wire(d.crank_task_nm)},
            {"crank_task_shortfall_nm",
             Wire(d.crank_task_shortfall_nm)},
            {"solution_excitation_nm",
             wire_array(std::span<const double>(
                 d.solution_excitation_nm))}};
}

// support_diagnostics_dict — {'stance': {'front','rear'}}.
WireObject wire_support_targets(const rider::SupportDiagnostics &d) {
    if (!d.present)
        return {};
    if (d.legacy) {
        auto const value = bike::rider::encode_support_diagnostics(*d.legacy);
        auto wire = vread::wire_from_value(value, "channels.rider_support_targets");
        return std::get<WireObject>(std::move(wire.value));
    }
    return {{"stance",
             Wire(WireObject{{"front", Wire(d.stance_front)},
                             {"rear", Wire(d.stance_rear)}})}};
}

// effort_diagnostics_dict — finalize keys first, solved_effort additions
// appended in the oracle's dict-update order.
WireObject wire_effort(const rider::EffortDiagnostics &d) {
    if (!d.present)
        return {};
    WireObject out;
    out.emplace_back("rider_active_request_nm",
                     Wire(wire_named_reals(d.active_request_nm)));
    out.emplace_back("rider_active_delivered_nm",
                     Wire(wire_named_reals(d.active_delivered_nm)));
    out.emplace_back("rider_positive_power_w",
                     Wire(d.positive_power_w));
    out.emplace_back("rider_passive_power_w",
                     Wire(d.passive_power_w));
    out.emplace_back("rider_activation_saturated",
                     Wire(d.activation_saturated));
    out.emplace_back("rider_strength_limited",
                     Wire(wire_names(d.strength_limited)));
    if (d.budget_present)
        out.emplace_back("rider_effort_budget_exceeded",
                         Wire(d.budget_exceeded));
    if (d.power_limit_present)
        out.emplace_back("rider_active_positive_power_limit_w",
                         wire_opt(d.positive_power_limit_w));
    if (d.observation_present)
        out.emplace_back("rider_effort_observation",
                         Wire(d.observation));
    if (d.joint_positive_power_w.has_value())
        out.emplace_back(
            "rider_joint_positive_power_w",
            Wire(wire_named_reals(*d.joint_positive_power_w)));
    if (d.joint_power_violations.has_value())
        out.emplace_back("rider_joint_power_violations",
                         Wire(wire_names(*d.joint_power_violations)));
    if (d.joint_speed_violations.has_value())
        out.emplace_back("rider_joint_speed_violations",
                         Wire(wire_names(*d.joint_speed_violations)));
    if (d.positive_work_step_j.has_value())
        out.emplace_back("rider_positive_work_step_j",
                         Wire(*d.positive_work_step_j));
    if (d.passive_work_step_j.has_value())
        out.emplace_back("rider_passive_work_step_j",
                         Wire(*d.passive_work_step_j));
    if (d.strength_violations.has_value())
        out.emplace_back("rider_strength_violations",
                         Wire(wire_names(*d.strength_violations)));
    return out;
}

// settle_welds' result dict — support/pedal/grip entries in insertion
// order, then the conditional 'crank_torque_nm' tail.
WireObject
wire_settled_welds(const writers::RiderSettleOutcome &settle) {
    WireObject out;
    for (const auto &[name, entry]: settle.entries)
        out.emplace_back(name, Wire(wire_settled_entry(entry)));
    if (settle.crank_torque_nm_appended)
        out.emplace_back("crank_torque_nm",
                         Wire(settle.crank_torque_nm));
    return out;
}

// compiled_center_of_mass (mass_properties.py:7-13) — the column sum also
// shared by mass_observations. The oracle is
// np.sum(masses[:, None] * xipos, axis=0): reducing the strided outer axis
// of a C-order (nbody, 3) product is a plain sequential per-column fold in
// numpy, not the contiguous pairwise reduction. Verified bitwise against
// numpy 2.5.2 for nbody = 2..1000.
std::array<double, 3> compiled_com(const mjModel *m, const mjData *d) {
    const std::size_t nbody = static_cast<std::size_t>(m->nbody);
    const auto body_mass = buf(m->body_mass, m->nbody);
    const auto xipos = buf(d->xipos, 3 * m->nbody);
    const double total = numpy_sum(body_mass);
    if (!(total > 0.))
        throw std::invalid_argument("model has no positive physical mass");
    std::array<double, 3> com{};
    for (std::size_t k = 0; k < 3; ++k) {
        double sum = 0.;
        for (std::size_t i = 0; i < nbody; ++i)
            sum += body_mass[i] * xipos[3 * i + k];
        com[k] = sum / total;
    }
    return com;
}

// com_observations (physical_energy.py:45-59) — the decimated-step mass
// channel: {mass_kg, com_m} only, no momentum/KE/PE keys and no
// engine::subtree_vel/mul_m calls. Same formulas mass_observations uses,
// so a com_m column is identical across full and decimated intervals.
WireObject com_observations(const mjModel *m, const mjData *d) {
    const auto body_mass = buf(m->body_mass, m->nbody);
    const double total = numpy_sum(body_mass);
    if (!(total > 0.))
        throw std::invalid_argument("model has no physical mass");
    WireObject out;
    out.emplace_back("mass_kg", Wire(total));
    out.emplace_back("com_m", wire_vec3(compiled_com(m, d)));
    return out;
}

} // namespace

// ---- construction ------------------------------------------------------

PhysicalStep::PhysicalStep(
    Stepper &stepper, StepConfig config,
    std::vector<std::array<double, 2>> vertices, StaticBrake brake,
    const std::optional<rider::SpindlePose> &rider_pose,
    const std::optional<rider::SpindleConfig> &rider_config)
    : stepper_(&stepper), config_(config),
      vertices_(std::move(vertices)),
      road_profile_(vertices_),
      rider_intent_(config_.intent, config_.timestep_s),
      brake_(brake),
      control_clock_(config_.timestep_s, config_.control_period_s),
      contact_query_(stepper_->model(), kContactDropoutSteps),
      probe_query_(stepper_->model(), kContactDropoutSteps),
      filters_{GroundedFilter{config_.monitors.grounded_hold_s},
               GroundedFilter{config_.monitors.grounded_hold_s}},
      balance_monitor_(config_.monitors.balance_floor_mps,
                       config_.monitors.balance_dwell_s,
                       config_.monitors.balance_grace_s),
      crash_detector_(stepper.model()),
      accumulator_(stepper_->model()->nv) {
    const mjModel *m = stepper_->model();
    if (rider_pose.has_value() != rider_config.has_value())
        throw std::invalid_argument(
            "rider pose and rider config must arrive together");
    if (rider_pose.has_value())
        rider_control_.emplace(m, *rider_pose, *rider_config,
                               config_.crank_length_m);
    drive_ctrl_adr_ = mj_name2id(m, mjOBJ_ACTUATOR, "rear_drive");
    cg_site_id_ = mj_name2id(m, mjOBJ_SITE, "site_CG");
    frame_body_ = mj_name2id(m, mjOBJ_BODY, "frame");
    root_x_qposadr_ = joint_qposadr(m, "root_x");
    root_x_dofadr_ = joint_dofadr(m, "root_x");
    root_pitch_qposadr_ = joint_qposadr(m, "root_pitch");
    shock_stroke_jnt_ = mj_name2id(m, mjOBJ_JOINT, "shock_stroke");
    crank_spin_qposadr_ = joint_qposadr(m, "crank_spin");
    crank_spin_dof_ = joint_dofadr(m, "crank_spin");
    // sensor_channels reads the wheel/crank encoder dofs and the
    // _wheel_bodies lookup unconditionally (sim.address); the crash
    // detector's own ctor resolves the same root joints again like the
    // oracle's CrashDetector(model).
    wheel_bodies_ = {mj_name2id(m, mjOBJ_BODY, "front_wheel"),
                     mj_name2id(m, mjOBJ_BODY, "rear_wheel")};
    wheel_spin_dofs_ = {joint_dofadr(m, "front_wheel_spin"),
                        joint_dofadr(m, "rear_wheel_spin")};
    accel_sensor_adr_ = sensor_adr(m, "sensor_frame_accel");
    gyro_sensor_adr_ = sensor_adr(m, "sensor_frame_gyro");
    crash_geoms_ = crash_geom_ids(m);
    // native_reference has no applied tire writer: its solved contact
    // rows are read by RuntimeContactQuery after the engine solve.
    // Per-wheel staging bound: the writer reserves its published patch
    // capacity at construction (station_count distributed, one contact
    // patch compliant — snapshot_storage() exposes the committed slots
    // it was reserved on). Sizing the resistance-input staging to that
    // bound means the per-step assign reuses capacity instead of
    // reallocating — the same construction-sized discipline the
    // writers' marked commit uses. The durable snapshots_ slots mirror
    // the oracle's `self.snapshots = {}` instead: disengaged until the
    // first publish, which engages them with this same bound reserved
    // (see refresh_contacts) so restore() can re-disengage them without
    // losing the sizing discipline.
    // The committed storage outlives the borrowed view (it lives on
    // the writer) — the named view is also what GCC's
    // -Wdangling-reference needs to see.
    if (stepper_->has_tire()) {
        const TireWriterView tire_view = stepper_->tire_writer();
        const std::array<TireSnapshot, 2> &tire_storage = tire_view.snapshot_storage();
        for (std::size_t i = 0; i < 2; ++i) {
            const std::size_t bound = tire_storage[i].patches.capacity();
            patch_loads_[i].reserve(bound);
            if (bound > patch_working_capacity_[i]) {
                // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) contiguous bool buffer — vector<bool> is a bitset
                patch_working_[i] = std::make_unique<bool[]>(bound);
                patch_working_capacity_[i] = bound;
            }
        }
    }
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    component_out_.assign(nv, 0.);
    mul_scratch_.assign(nv, 0.);
    measurement_decimation_ = config_.measurement_decimation;
}

// ---- forward/step deduplication ----------------------------------------
// posvel_fresh() is the single decision point for the VEL-skip fast path:
// the Stepper flag is ANDed with the drivetrain's live-model write bit so a
// transmission commit (wrap_prm / tendon_range / mj_setConst) made since
// the last check still reaches the guard even when no caller flagged it.

std::string_view PhysicalStep::drive_mode() const {
    return stepper_->drive().config().drive_mode;
}

bool PhysicalStep::posvel_fresh() {
    if (stepper_->has_drive() &&
        stepper_->drive().take_model_dirty())
        stepper_->note_posvel_stale();
    return stepper_->posvel_fresh();
}

void PhysicalStep::refresh_forward(mjModel *m, mjData *d) {
    if (posvel_fresh())
        engine::forward_skip(m, d);
    else
        engine::forward(m, d);
    stepper_->note_posvel_fresh();
    acc_inputs_valid_ = false;
}

void PhysicalStep::ensure_posvel_fresh(mjModel *m, mjData *d) {
    if (!posvel_fresh()) {
        engine::forward(m, d);
        stepper_->note_posvel_fresh();
    }
    acc_inputs_valid_ = false;
}

namespace {
std::size_t acc_input_count(const mjModel *m) {
    return static_cast<std::size_t>(m->nv) +
           static_cast<std::size_t>(m->nbody) * 6 +
           static_cast<std::size_t>(m->nu) + static_cast<std::size_t>(m->na) +
           1 + static_cast<std::size_t>(m->nq) + static_cast<std::size_t>(m->nv);
}
// Bitwise equality of two double views: -0.0 vs 0.0 and NaN payloads count
// as differences, exactly as the engine would see them.
bool same_bits(std::span<const double> a, std::span<const double> b) {
    return std::ranges::equal(a, b, [](double x, double y) {
        return std::bit_cast<std::uint64_t>(x) == std::bit_cast<std::uint64_t>(y);
    });
}
// The acc-stage input views in snapshot order.
std::array<std::span<const double>, 7> acc_input_views(const mjModel *m,
                                                      const mjData *d) {
    return {buf(d->qfrc_applied, m->nv), buf(d->xfrc_applied, m->nbody * 6),
            buf(d->ctrl, m->nu),         buf(d->act, m->na),
            std::span<const double>(&d->time, 1),
            buf(d->qpos, m->nq),         buf(d->qvel, m->nv)};
}
} // namespace

void PhysicalStep::snapshot_acc_inputs(const mjModel *m, const mjData *d) {
    acc_inputs_.resize(acc_input_count(m));
    const std::span<double> out(acc_inputs_);
    std::size_t at = 0;
    for (const auto view : acc_input_views(m, d)) {
        std::ranges::copy(view, out.subspan(at, view.size()).begin());
        at += view.size();
    }
    acc_inputs_valid_ = true;
}

bool PhysicalStep::acc_inputs_unchanged(const mjModel *m,
                                        const mjData *d) const {
    if (!acc_inputs_valid_ || acc_inputs_.size() != acc_input_count(m))
        return false;
    const std::span<const double> saved(acc_inputs_);
    std::size_t at = 0;
    for (const auto view : acc_input_views(m, d)) {
        if (!same_bits(saved.subspan(at, view.size()), view))
            return false;
        at += view.size();
    }
    return true;
}

// ---- apply_forces (physical_runtime.py:219-349) -------------------------
// The staged force assembly: writer evaluation order IS the arithmetic.

RuntimeContacts
PhysicalStep::apply_forces(bool active, bool advance, double front,
                           double rear,
                           const std::optional<std::vector<double>> &external,
                           const RideControl &caller_control,
                           std::optional<bool> braking_opt) {
    // The staged body sits in its own frame so the mutate lambda stays a
    // thin call; the debug/asan frame-budget sweep bounds each function.
    return stepper_->mutate([&]() -> RuntimeContacts {
        return apply_forces_body(active, advance, front, rear, external,
                                 caller_control, braking_opt);
    });
}

RuntimeContacts
PhysicalStep::apply_forces_body(
    bool active, bool advance, double front, double rear,
    const std::optional<std::vector<double>> &external,
    const RideControl &caller_control, std::optional<bool> braking_opt) {
    {
        mjModel *m = stepper_->model();
        mjData *d = stepper_->data();
        control_validate_for(caller_control, rider_control_.has_value(), drive_mode());
        const double dt = config_.timestep_s;
        const bool control_tick =
            !advance || initializing_ || control_clock_.is_tick(step_);
        if (rider_control_.has_value() && control_tick) {
            const auto xpos = buf(d->xpos, 3 * m->nbody);
            wheel_x_m_ = {xpos[3 * static_cast<std::size_t>(
                              wheel_bodies_[0])],
                          xpos[3 * static_cast<std::size_t>(
                              wheel_bodies_[1])]};
            rider_state_ = rider_kinematic_state(
                m, d, road_profile_, wheel_x_m_, config_.road_lookahead_m);
        }
        const bool automatic_effort =
            active && config_.intent.enabled &&
            !caller_control.human_torque_nm.has_value() &&
            caller_control.rider_enabled;
        RideControl control = caller_control;
        if (config_.intent.enabled) {
            const auto &road = rider_state_.has_value()
                                   ? rider_state_->road
                                   : std::vector<RoadSample>{};
            control = rider_intent_.resolve(
                control, intent_signals_, step_,
                road_grade_for_posture(road), road_grade_preview(road),
                rider_control_.has_value() ? rider_control_->lean_limit_rad()
                                           : std::optional<double>{},
                active, advance);
        }
        if (advance)
            applied_control_ = control;
        const bool braking =
            braking_opt.value_or(front > 0. || rear > 0.);

        std::ranges::fill(wbuf(d->qfrc_applied, m->nv), 0.);
        std::ranges::fill(wbuf(d->xfrc_applied, m->nbody * 6), 0.);
        std::ranges::fill(wbuf(d->ctrl, m->nu), 0.);
        // dof_frictionloss is a position-stage model input — a bound that
        // actually changed invalidates the cached makeConstraint rows.
        if (brake_.apply(m, front, rear))
            stepper_->note_posvel_stale();
        // prepare_pedaling reads the LAST committed rear contact — the
        // durable contacts_ (previous step's final refresh or bootstrap).
        const drivetrain::RideControl drive_control{
            .motor_torque_nm = control.motor_torque_nm,
            .motor_limit_nm = control.motor_limit_nm,
            .human_torque_nm = control.human_torque_nm,
            .rider_enabled = control.rider_enabled};
        const drivetrain::PedalingState pedaling = stepper_->drive().prepare(
            drive_control, dt, braking, active, advance,
            contacts_.rear_controller_grounded.value_or(false),
            contacts_.rear_slip_mps,
            automatic_effort ? control.human_torque_nm
                             : std::optional<double>{});
        // Oracle: the apply_forces entry forward runs only while
        // `initializing` — init callers mutate qpos/qvel or
        // constraint-defining model constants between apply_forces calls
        // and rely on this pass to refresh poses/efc_pos; the running
        // loop enters with derived state already current from the
        // post-step forward, so the zero-input solve here would feed
        // only warmstarts (physical_runtime.py:262-271). Outside init a
        // brake or transmission model write (a pos/vel-stage input)
        // still falls back to the full stage.
        if (initializing_)
            refresh_forward(m, d);
        else
            ensure_posvel_fresh(m, d);

        accumulator_.clear();
        const std::span<const std::string_view> susp_names =
            stepper_->suspension().component_names();
        stepper_->suspension().components_into(
            d, std::span{suspension_views_}.first(susp_names.size()));
        for (std::size_t i = 0; i < susp_names.size(); ++i)
            accumulator_.add(susp_names[i], suspension_views_[i].values);
        if (external.has_value())
            accumulator_.add("external", *external);
        // tires: the probe path detaches brush state inside the writer —
        // compute or probe, never both (tire_forces.py:108-113).
        if (stepper_->has_tire()) {
            if (advance)
                stepper_->tire_writer().compute_into(d, dt, component_out_);
            else
                stepper_->tire_writer().probe_compute_into(d, dt, component_out_);
            accumulator_.add("tires", component_out_);
        }
        // OPEN FINDING F3: when the tire writer is absent the oracle still
        // evaluates resistance from the solved contact snapshots
        // (physical_runtime.py:385-387 — `self.tire is None`). This port
        // runs resistance only inside the tire-writer path, so the
        // native_reference route drops aerodynamic/rolling losses.
        if (active && stepper_->has_tire()) {
            // The resistance input reads the POST-compute committed
            // state — snapshots_committed() is the same publication gate
            // snapshots() projects into its optional pair, so the
            // borrowed committed/probe storage holds exactly what the
            // copying accessor would have returned. Reading it in place
            // avoids the per-step snapshot copy (the distributed
            // backend's patches vectors plus its non-SSO backend
            // string).
            const TireWriterView tires = stepper_->tire_writer();
            const bool committed =
                advance ? tires.snapshots_committed()
                        : tires.probe_committed();
            const std::array<TireSnapshot, 2> &snaps =
                advance ? tires.snapshot_storage()
                        : tires.probe_snapshot_storage();
            std::array<TireSideInput, 2> side_input{};
            for (std::size_t i = 0; i < 2; ++i) {
                const TireSnapshot &snap = snaps[i];
                const auto *patches =
                    committed ? &snap.patches : nullptr;
                const std::size_t n =
                    patches != nullptr ? patches->size() : 0;
                patch_loads_[i].assign(n, 0.);
                if (n > patch_working_capacity_[i]) {
                    // Defensive regrowth only — construction sized the
                    // buffer to the writer's patch bound, so this can
                    // never trip on a valid snapshot.
                    // NOLINTNEXTLINE(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) contiguous bool buffer — vector<bool> is a bitset
                    patch_working_[i] = std::make_unique<bool[]>(n);
                    patch_working_capacity_[i] = n;
                }
                const std::span<bool> working{
                    std::views::counted(patch_working_[i].get(),
                                        static_cast<std::ptrdiff_t>(n))};
                for (std::size_t p = 0; p < n; ++p) {
                    patch_loads_[i][p] = (*patches)[p].normal_load_n;
                    working[p] = (*patches)[p].working_surface;
                }
                side_input[i] = {
                    .patch_loads = std::span<const double>(
                        std::views::counted(patch_loads_[i].data(),
                                            static_cast<std::ptrdiff_t>(n))),
                    .patch_working = working,
                    .effective_radius_m =
                        committed ? snap.effective_radius_m : 0.};
            }
            const std::span<const std::string_view> res_names =
                stepper_->resistance().component_names();
            stepper_->resistance().components_into(
                d, side_input[0], side_input[1],
                std::span{resistance_views_}.first(res_names.size()));
            for (std::size_t i = 0; i < res_names.size(); ++i)
                accumulator_.add(res_names[i],
                                 resistance_views_[i].values);
        }
        if (stepper_->has_rider_forces() &&
            stepper_->rider_forces().active()) {
            stepper_->rider_forces().compute_into(d, component_out_);
            accumulator_.add("seated_interfaces", component_out_);
        }
        if (rider_control_.has_value()) {
            stepper_->rider_contacts().compute_qfrc_into(
                d, dt, advance, /*detailed=*/true, component_out_);
            accumulator_.add("rider_interfaces", component_out_);
        }
        double sensed = 0.;
        if (rider_control_.has_value()) {
            if (advance)
                sensed = stepper_->rider_contacts()
                             .state()
                             .delivered_crank_torque_nm;
            else {
                const auto &probe = stepper_->rider_contacts().probe();
                sensed = probe.has_value()
                             ? probe->delivered_crank_torque_nm
                             : stepper_->rider_contacts()
                                   .state()
                                   .delivered_crank_torque_nm;
            }
        }
        const drivetrain::TickInputs tick_inputs{
            .control = drive_control,
            .dt = dt,
            .speed = buf(d->qvel, m->nv)[static_cast<std::size_t>(
                root_x_dofadr_)],
            .sensed = sensed,
            .braking = braking,
            .active = active,
            .advance = advance,
            .contact = true,
            .pedaling = pedaling,
            .slip = std::nullopt};
        const auto tick = stepper_->drive().stage_components(tick_inputs);
        for (std::size_t i = 0; i < tick.components.size(); ++i)
            accumulator_.add(tick.component_names[i],
                             tick.components[i].values);
        stepper_->drive().commit(tick);
        if (rider_control_.has_value()) {
            std::optional<double> crank_goal = pedaling.target_phase_rad;
            if (!active && stepper_->drive().transmission_storage() != nullptr)
                crank_goal = stepper_->drive().config().crank_phase_rad;
            rider::RiderCommand command;
            command.mean_crank_torque_nm =
                stepper_->drive().config().drive_mode == "articulated_effort"
                    ? pedaling.effort_nm
                    : 0.;
            command.enabled = control.rider_enabled;
            command.posture = control.posture.value_or(rider::RiderPosture{});
            command.crank_target_phase_rad = crank_goal;
            command.crank_target_rate_rad_s =
                control.crank_target_rate_rad_s.value_or(
                    pedaling.target_rate_rad_s);
            accumulator_.add("rider_joint_envelope",
                             rider_control_->envelope_forces(d).first);
            rider::NamedEntries<double> torques;
            if (control_tick) {
                const auto total = accumulator_.total();
                std::ranges::copy(
                    total,
                    wbuf(d->qfrc_applied, m->nv).begin());
                ensure_posvel_fresh(m, d);
                torques = engaged(rider_control_).compute(
                    d, command, advance,
                    advance && !initializing_ ? control_clock_.period_s : dt,
                    !active, stepper_->rider_contacts().state().enabled,
                    // OPEN FINDING F5: pedal_recovery — the oracle passes
                    // cfg.seated_climb.enabled (physical_runtime.py:367);
                    // this port passes the active-tick flag instead.
                    active && !initializing_);
                if (advance) {
                    control_clock_.hold(torques);
                    if (!initializing_)
                        held_rider_terms_ = engaged(rider_control_).state().last_terms;
                }
            } else {
                torques = control_clock_.held();
            }
            engaged(rider_control_).write(d, torques);
        }
        const auto total = accumulator_.total();
        std::ranges::copy(total,
                          wbuf(d->qfrc_applied, m->nv).begin());
        refresh_forward(m, d);
        snapshot_acc_inputs(m, d);
        const RuntimeContacts contacts =
            refresh_contacts(false, std::nullopt, active && advance);
        if (stepper_->has_cruise()) {
            // sim.cruise.torque_nm = 0. — the oracle clears the scalar
            // unconditionally before the ideal_speed_control write below.
            CruiseState cs = stepper_->cruise().state();
            cs.torque_nm = 0.;
            stepper_->cruise().set_state(cs);
        }
        if (active &&
            stepper_->drive().config().drive_mode == "ideal_speed_control") {
            // OPEN FINDING F2: capabilities.cpp admits this drive mode and
            // the oracle calls cruise.compute() here
            // (physical_runtime.py:389-394, braking suppresses positive
            // torque). The cruise-controller port is pending, so the
            // branch intentionally commands zero torque for now.
            wbuf(d->ctrl, m->nu)[static_cast<std::size_t>(
                drive_ctrl_adr_)] = 0.;
        }
        const auto final_total = accumulator_.total();
        std::ranges::copy(final_total,
                          wbuf(d->qfrc_applied, m->nv).begin());
        return contacts;
    }
}

// ---- _advance_physics (physical_runtime.py:561-700) --------------------

RawStep PhysicalStep::advance_physics(
    double front, double rear,
    const std::optional<std::vector<double>> &external,
    const RideControl &control) {
    // The step body sits in its own frame so the mutate lambda stays a
    // thin call; the debug/asan frame-budget sweep bounds each function.
    return stepper_->mutate([&]() -> RawStep {
        return advance_body(front, rear, external, control);
    });
}

RawStep PhysicalStep::advance_body(
    double front, double rear,
    const std::optional<std::vector<double>> &external,
    const RideControl &control) {
        if (!research_accounting_valid_)
            throw std::runtime_error(
                "reset is required before physical accounting");
        control_validate_for(control, rider_control_.has_value(), drive_mode());
        scalar_field(front, "front brake demand");
        scalar_field(rear, "rear brake demand");
        mjModel *m = stepper_->model();
        mjData *d = stepper_->data();
        const std::size_t nv = static_cast<std::size_t>(m->nv);
        std::optional<std::vector<double>> ext;
        if (external.has_value()) {
            if (external->size() != nv)
                throw std::invalid_argument("invalid generalized force");
            for (const double v: *external)
                if (!std::isfinite(v))
                    throw std::invalid_argument("invalid generalized force");
            ext = *external;
        }
        const double t = d->time;
        std::vector<double> q = buffer_copy(d->qpos, m->nq);
        std::vector<double> v = buffer_copy(d->qvel, m->nv);
        const bool braking = front > 0. || rear > 0.;
        const double hold = rollback_brake_demand(
            buf(d->qvel, m->nv)[static_cast<std::size_t>(root_x_dofadr_)],
            &control);
        if (hold > 0.) {
            front = std::max(front, hold);
            rear = std::max(rear, hold);
        }
        (void)apply_forces(true, true, front, rear, ext, control, braking);

        std::optional<writers::RiderPreparedAttachments> prepared;
        if (rider_control_.has_value())
            prepared = stepper_->rider_contacts().prepare_attachment_raw(d);
        // Oracle (physical_runtime.py:596-603): the energy/momentum
        // solve belongs to full telemetry intervals only; every raw of a
        // period reports the period's closing measurement. Decimated
        // intervals keep the compiled mass and CoM rows the per-interval
        // truth consumers read, at kinematic cost only. `full` is
        // computed once here so raw.full and the measurement gate stay
        // one value (step_ is committed only at publish_raw's tail).
        const bool full =
            step_ % measurement_decimation_ == 0 ||
            (step_ + 1) % control_clock_.steps_per_period == 0;
        WireObject mass0 = full ? std::move(mass_observations(m, d).wire)
                                : com_observations(m, d);
        last_force_sample_ = ForceSample{.time_s = t,
                                         .qpos = q,
                                         .qvel = v,
                                         .components =
                                             accumulator_.components()};
        ForceMap components = accumulator_.components();
        const std::span<const mjWarningStat> warnings(d->warning);
        const std::array<int, 3> warning_counts{
            warnings[static_cast<std::size_t>(mjWARN_BADQPOS)].number,
            warnings[static_cast<std::size_t>(mjWARN_BADQVEL)].number,
            warnings[static_cast<std::size_t>(mjWARN_BADQACC)].number};
        // Oracle: mj_step = checks + mj_forward + checks + integrator.
        // step_skip replays that sequence verbatim with the pos/vel
        // stages skipped; the same freshness gate applies — a stale
        // cache gets the untouched full step. Integration invalidates
        // the flag for the next interval.
        if (posvel_fresh()) {
            // R2: identical acc-stage inputs since the force-phase forward
            // mean mj_step's own forward would leave the same bits — skip it.
            if (acc_inputs_unchanged(m, d))
                engine::step_skip_fresh(m, d);
            else
                engine::step_skip(m, d);
        } else {
            engine::step(m, d);
        }
        acc_inputs_valid_ = false;
        stepper_->note_posvel_stale();
        return solve_tail({.t = t,
                           .q = std::move(q),
                           .v = std::move(v),
                           .front = front,
                           .rear = rear,
                           .braking = braking,
                           .hold = hold,
                           .control = &control,
                           .prepared = std::move(prepared),
                           .mass0 = std::move(mass0),
                           .full = full,
                           .components = std::move(components),
                           .warning_counts = warning_counts});
}

RawStep PhysicalStep::solve_tail(SolvedInputs in) {
        const mjModel *m = stepper_->model();
        mjData *d = stepper_->data();
        const std::size_t nv = static_cast<std::size_t>(m->nv);
        const double dt = config_.timestep_s;
        const double t = in.t;
        const std::vector<double> &q = in.q;
        const std::vector<double> &v = in.v;
        const double front = in.front, rear = in.rear;
        const double hold = in.hold;
        std::optional<writers::RiderPreparedAttachments> prepared =
            std::move(in.prepared);
        ForceMap components = std::move(in.components);
        const std::array<int, 3> warning_counts = in.warning_counts;
        const std::span<const mjWarningStat> warnings(d->warning);
        in.constraint_powers =
            numerical_constraint_powers(m, d, v, mul_scratch_);
        static constexpr std::array<int, 3> watched = {
            mjWARN_BADQPOS, mjWARN_BADQVEL, mjWARN_BADQACC};
        static constexpr std::array<const char *, 3> watched_names = {
            "mjWARN_BADQPOS", "mjWARN_BADQVEL", "mjWARN_BADQACC"};
        for (std::size_t i = 0; i < 3; ++i)
            if (warnings[static_cast<std::size_t>(watched[i])].number >
                warning_counts[i])
                throw std::runtime_error(
                    std::string("MuJoCo numerical failure: ") +
                    watched_names[i]);
        // efc_force and poses still belong to this solved interval.
        if (rider_control_.has_value()) {
            in.settle = stepper_->rider_contacts().settle_welds(
                d, writers::RiderIntervalState{q, v}, true,
                &engaged(prepared));
            in.raw_map = in.settle.measurement.raws;
            // settle_welds' error vector already joins the prepared
            // block's ':unobservable_attachment_wrench' entries like the
            // oracle's triple return — do not append prepared->errors a
            // second time.
            in.attachment_errors = in.settle.measurement.errors;
        }
        in.contact_crash = physical_contact_crash(
            m, d, crash_geoms_.catch_ids, crash_geoms_.terrain_ids,
            crash_geoms_.rider_ids);
        const auto transmission_span = stepper_->drive().settle();
        std::vector<double> transmission(transmission_span.begin(),
                                         transmission_span.end());
        // One materialized telemetry map per interval — settle() is the
        // last commit into sim.drive.diagnostics() before the three
        // read-only consumers below (sensors, loss lanes, in.drive), so
        // the map can't diverge between them.
        const drivetrain::Diagnostics drive_last =
            stepper_->drive().diagnostics(false).to_map();
        in.sensors = sensor_channels(v, drive_last);
        for (auto &[name, f]: actuator_components(m, d))
            ordered_set(components, name, std::move(f));
        if (rider_control_.has_value()) {
            auto passive = zeros(m);
            const auto qfrc_passive = buf(d->qfrc_passive, m->nv);
            for (const auto &j: rider_control_->joints())
                passive[static_cast<std::size_t>(j.dof_adr)] =
                    qfrc_passive[static_cast<std::size_t>(j.dof_adr)];
            ordered_set(components, "rider_passive_damping",
                        std::move(passive));
        }
        ForceMap constraints = constraint_components(m, d, mul_scratch_);
        if (stepper_->drive().transmission_storage() != nullptr) {
            // constraints['joint_limits'] -= transmission — the group
            // always exists (constraint_components emits all three
            // unconditionally); the oracle's -= would KeyError on a miss.
            auto &jl = require_component_mut(constraints, "joint_limits");
            for (std::size_t i = 0; i < nv; ++i)
                jl[i] -= transmission[i];
            ordered_set(constraints, "ideal_transmission", transmission);
        }
        auto shock_limit = shock_joint_limit_qfrc(
            m, d, shock_stroke_jnt_, mul_scratch_);
        {
            auto &jl =
                require_component_mut(constraints, "joint_limits");
            for (std::size_t i = 0; i < nv; ++i)
                jl[i] -= shock_limit[i];
        }
        ordered_set(constraints, "shock_solver_limit", shock_limit);
        for (auto &[name, f]: constraints)
            ordered_set(components, name, std::move(f));
        const auto brake_components = brake_.solved_components(m, d);
        ordered_set(components, "front_static_brake", brake_components.first);
        ordered_set(components, "rear_static_brake",
                    brake_components.second);
        std::vector<double> passive = buffer_copy(d->qfrc_passive, m->nv);
        ordered_set(components, "engine_passive", std::move(passive));
        if (rider_control_.has_value())
            if (auto *ep = ordered_at(components, "engine_passive"))
                if (const auto *rp =
                        ordered_at(components, "rider_passive_damping"))
                    for (std::size_t i = 0; i < nv; ++i)
                        (*ep)[i] -= (*rp)[i];
        // The constraint snapshot reuses the already-computed rows —
        // solved_components is deterministic on the same solved state.
        ConstraintSnapshot snap{.interval_start_s = t,
                                .interval_end_s = d->time,
                                .qvel_start = v,
                                .components = {}};
        snap.components.emplace_back("shock_solver_limit", shock_limit);
        snap.components.emplace_back("front_static_brake",
                                     brake_components.first);
        snap.components.emplace_back("rear_static_brake",
                                     brake_components.second);
        last_constraint_snapshot_ = std::move(snap);
        contacts_ = refresh_contacts(true, t, true, v);
        in.tires = tire_channels(snapshots_, v);
        if (config_.intent.enabled) {
            WireObject signals_in = in.sensors;
            wire_set(signals_in, "tires", Wire(in.tires));
            update_intent_signals(signals_in);
        }
        in.loss_step = loss_increment(components, v, dt, drive_last);
        if (rider_control_.has_value())
            in.rider_diag =
                stepper_->rider_contacts().state().diagnostics;
        const double crank_phase = q[static_cast<std::size_t>(
            crank_spin_qposadr_)];
        in.drive = std::get<WireObject>(
            std::move(wire_diagnostics(drive_last).value));
        // dict(last, **kw) semantics — wire_set keeps the insertion
        // position when a telemetry lane name collides.
        wire_set(in.drive, "crank_phase_rad", Wire(crank_phase));
        wire_set(in.drive, "front_brake_demand", Wire(front));
        wire_set(in.drive, "rear_brake_demand", Wire(rear));
        wire_set(in.drive, "rollback_brake_demand", Wire(hold));
        const auto connect_rows = connect_equality_rows(m, d);
        const auto efc_pos = buf(d->efc_pos, d->nefc);
        for (const int row: connect_rows)
            in.linkage_error = std::max(
                in.linkage_error,
                std::abs(efc_pos[static_cast<std::size_t>(row)]));
        // last_constraint_snapshot.components['shock_solver_limit'] @ v —
        // the row was just emplaced above; the oracle's index is total.
        in.shock_limit_power = dot(
            require_component(last_constraint_snapshot_.components,
                              "shock_solver_limit"),
            v);
        in.electrical_power_w =
            diag_wire_default_zero(drive_last, "electrical_power_w");
        in.solved_actuator = buffer_copy(d->actuator_force, m->nu);
        in.solved_passive = buffer_copy(d->qfrc_passive, m->nv);
        // Deliberately a FULL forward: this refreshes the pos/vel caches
        // for the integrated state, seeding the next interval's skip
        // eligibility (and the endpoint kinematics the oracle records).
        engine::forward(m, d);
        stepper_->note_posvel_fresh();
        for (const double value: buf(d->qpos, m->nq))
            if (!std::isfinite(value))
                throw std::runtime_error(
                    "non-finite physical simulation state");
        for (const double value: buf(d->qvel, m->nv))
            if (!std::isfinite(value))
                throw std::runtime_error(
                    "non-finite physical simulation state");
        // components was moved out of `in` at the top of this frame so the
        // solved rows could be assembled in place; publish_raw reads the
        // same assembled map through the struct — hand it back.
        in.components = std::move(components);
        return publish_raw(std::move(in));
}

// ---- publish stage (physical_runtime.py:662-700) ----------------------

RawStep PhysicalStep::publish_raw(SolvedInputs in) {
        const mjModel *m = stepper_->model();
        const mjData *d = stepper_->data();
        const std::size_t nv = static_cast<std::size_t>(m->nv);
        const double t = in.t;
        const std::vector<double> &q = in.q;
        const std::vector<double> &v = in.v;
        const double front = in.front, rear = in.rear;
        const bool braking = in.braking;
        const RideControl &control = *in.control;
        const WireObject &mass0 = in.mass0;
        ForceMap components = std::move(in.components);
        const writers::RiderSettleOutcome &settle = in.settle;
        const std::optional<std::string> &contact_crash = in.contact_crash;
        const WireObject &sensors = in.sensors;
        const WireObject &tires = in.tires;
        WireObject drive = std::move(in.drive);
        const double linkage_error = in.linkage_error;
        const double shock_limit_power = in.shock_limit_power;
        const double loss_step = in.loss_step;
        // Oracle (physical_runtime.py:667): `energy_state(self) if full
        // else (None,None,None)` — the endpoint momentum/elastic solve
        // runs on measured intervals only; unmeasured raws carry the
        // period's closing values (period close mirrors _close_period).
        std::optional<MassObservations> mass;
        std::optional<WireObject> elastic;
        std::optional<double> total;
        if (in.full) {
            auto [measured_mass, measured_elastic, measured_total] =
                energy_state();
            mass = std::move(measured_mass);
            elastic = std::move(measured_elastic);
            total = measured_total;
        }
        wire_set(drive, "chain_power_w",
                 Wire(dot(require_component(components, "chain"), v)));
        {
            // components['freehub'] + components.get('ideal_transmission',
            // zeros) — 'freehub' is a direct index (the pinned drivetrain
            // always emits it); 'ideal_transmission' merges when present.
            auto merged =
                require_component(components, "freehub");
            if (const auto *ideal =
                    ordered_at(components, "ideal_transmission"))
                for (std::size_t i = 0; i < nv; ++i)
                    merged[i] += (*ideal)[i];
            wire_set(drive, "freehub_power_w", Wire(dot(merged, v)));
        }
        wire_set(
            drive, "front_brake_power_w",
            Wire(dot(require_component(components, "front_static_brake"),
                     v)));
        wire_set(
            drive, "rear_brake_power_w",
            Wire(dot(require_component(components, "rear_static_brake"),
                     v)));
        wire_set(
            drive, "front_brake_torque_nm",
            Wire(require_component(components, "front_static_brake")
                     [static_cast<std::size_t>(brake_.front_dof())]));
        wire_set(
            drive, "rear_brake_torque_nm",
            Wire(require_component(components, "rear_static_brake")
                     [static_cast<std::size_t>(brake_.rear_dof())]));
        // sum(float(f@v) for n,f in components.items() if
        // n.startswith('act_rider_')) — CPython's float sum() is the
        // compensated loop, not a sequential +=.
        NeumaierSum human_active;
        const std::vector<double> *rider_passive = nullptr;
        for (const auto &[name, f]: components) {
            if (name.starts_with("act_rider_"))
                human_active.add(dot(f, v));
            if (name == "rider_passive_damping")
                rider_passive = &f;
        }
        const double human_active_w = human_active.total();
        // The human_joint term sums the generator a second time —
        // identical inputs, identical compensated total — then adds the
        // rider_passive_damping dot as a plain float add.
        wire_set(drive, "human_active_power_w", Wire(human_active_w));
        wire_set(
            drive, "human_joint_power_w",
            Wire(human_active_w +
                 (rider_passive != nullptr ? dot(*rider_passive, v)
                                           : 0.)));
        WireObject suspension;
        suspension.emplace_back(
            "fork_travel_m",
            Wire(q[static_cast<std::size_t>(
                stepper_->suspension().fork_qposadr())]));
        suspension.emplace_back(
            "shock_stroke_m",
            Wire(q[static_cast<std::size_t>(
                stepper_->suspension().shock_qposadr())]));
        suspension.emplace_back(
            "fork_velocity_mps",
            Wire(v[static_cast<std::size_t>(
                stepper_->suspension().fork_dofadr())]));
        suspension.emplace_back(
            "shock_velocity_mps",
            Wire(v[static_cast<std::size_t>(
                stepper_->suspension().shock_dofadr())]));
        suspension.emplace_back("shock_solver_limit_power_w",
                                Wire(shock_limit_power));
        suspension.emplace_back("linkage_closure_max_m",
                                Wire(linkage_error));
        WireObject gen_forces;
        for (const auto &[name, f]: components)
            if (name.starts_with("fork_") || name.starts_with("shock_"))
                gen_forces.emplace_back(
                    name,
                    Wire(f[static_cast<std::size_t>(
                        name.starts_with("fork")
                            ? stepper_->suspension().fork_dofadr()
                            : stepper_->suspension().shock_dofadr())]));
        suspension.emplace_back("generalized_force_components_n",
                                Wire(std::move(gen_forces)));
        // `in.full` is the measurement gate advance_body computed before
        // the interval's mass0 capture — one value drives both the mass
        // channel shape and raw.full (physical_runtime.py:600-601).
        WireObject channels;
        channels.emplace_back("tires", Wire(tires));
        channels.emplace_back("drive", Wire(std::move(drive)));
        channels.emplace_back("suspension", Wire(std::move(suspension)));
        channels.emplace_back("sensors", Wire(sensors));
        channels.emplace_back("mass", Wire(mass0));
        if (contact_crash.has_value())
            channels.emplace_back("contact_crash_cause",
                                  Wire(*contact_crash));
        else
            channels.emplace_back("contact_crash_cause", Wire(nullptr));
        channels.emplace_back("control", Wire(wire_control(applied_control_)));
        channels.emplace_back(
            "rider_balance",
            Wire(balance_channel(front, rear, braking,
                                 control.rider_enabled)));
        // state() deep-copies every named mirror — one snapshot feeds
        // the four rider reads below.
        const auto rider_state =
            rider_control_.has_value()
                ? std::optional(rider_control_->state())
                : std::nullopt;
        if (rider_state.has_value()) {
            const auto &st = *rider_state;
            WireObject torques;
            for (const auto &[name, value]: st.joint_torques_nm)
                torques.emplace_back(name, Wire(value));
            channels.emplace_back("rider_joint_torques",
                                  Wire(std::move(torques)));
            WireObject capacity;
            for (const auto &[name, value]: st.joint_capacity_nm)
                capacity.emplace_back(name, Wire(value));
            channels.emplace_back("rider_joint_capacity_nm",
                                  Wire(std::move(capacity)));
            channels.emplace_back("rider_lean_limit_rad",
                                  wire_opt(st.lean_limit_rad));
        }
        WireObject diagnostics;
        diagnostics.emplace_back(
            "rider",
            Wire(wire_contacts_diagnostics(in.rider_diag)));
        diagnostics.emplace_back("rider_welds",
                                 Wire(wire_settled_welds(settle)));
        diagnostics.emplace_back(
            "endpoint_mass",
            mass.has_value() ? Wire(mass->wire) : Wire(nullptr));
        if (config_.intent.enabled)
            diagnostics.emplace_back("rider_intent",
                                     Wire(wire_intent(rider_intent_.intent())));
        else
            diagnostics.emplace_back("rider_intent", Wire(nullptr));
        diagnostics.emplace_back(
            "inclination_rad",
            Wire(config_.intent.enabled
                     ? rider_intent_.policy().inclination_rad()
                     : 0.));
        if (rider_state.has_value()) {
            const auto &st = *rider_state;
            diagnostics.emplace_back(
                "rider_allocation",
                Wire(wire_allocation(st.allocation_diagnostics)));
            diagnostics.emplace_back("rider_ik_saturation",
                                     Wire(wire_named_bools(st.saturated_ik)));
            diagnostics.emplace_back(
                "rider_support_targets",
                Wire(wire_support_targets(st.support_diagnostics)));
        } else {
            diagnostics.emplace_back("rider_allocation", Wire(WireObject{}));
            diagnostics.emplace_back("rider_ik_saturation",
                                     Wire(WireObject{}));
            diagnostics.emplace_back("rider_support_targets",
                                     Wire(WireObject{}));
        }
        RawStep raw;
        raw.interval_id = step_;
        raw.time_s = t;
        raw.end_time_s = d->time;
        raw.qpos = q;
        raw.qvel = v;
        raw.components = std::move(components);
        raw.attachment_raw = std::move(in.raw_map);
        raw.actuator_force = std::move(in.solved_actuator);
        raw.qfrc_passive = std::move(in.solved_passive);
        raw.contact_truth = tires;
        raw.channels = std::move(channels);
        raw.diagnostics = std::move(diagnostics);
        raw.attachment_errors = std::move(in.attachment_errors);
        raw.rider_control_terms = held_rider_terms_;
        raw.numerical_constraint_power_w = in.constraint_powers;
        raw.loss_step_j = loss_step;
        raw.mechanical_energy_j = total;
        raw.elastic_energy_j = std::move(elastic);
        raw.battery_energy_j = stepper_->drive().battery_energy_j();
        raw.electrical_power_w = in.electrical_power_w;
        raw.invalid_controller =
            rider_state.has_value() &&
            rider_state->allocation_diagnostics.invalid_controller;
        if (rider_state.has_value()) {
            raw.effort_state = rider_state->effort_diagnostics;
            raw.effort_base = wire_effort(*raw.effort_state);
        } else {
            raw.effort_base = WireObject{};
        }
        raw.constraint_snapshot = last_constraint_snapshot_;
        raw.full = in.full;
        step_ += 1;
        if (contact_crash.has_value() && !crash_detector_.event().has_value())
            crash_detector_.set_event(CrashEvent{
                .cause = *contact_crash,
                .time_s = t,
                .position_m =
                    q[static_cast<std::size_t>(root_x_qposadr_)],
                .pitch_rad =
                    q[static_cast<std::size_t>(
                        root_pitch_qposadr_)]});
        crash_detector_.check(d, contacts_);
        update_compiled_com_marker();
        return raw;
}

// ---- _contacts (physical_runtime.py:160-194) ----------------------------
// The tire backend owns every wheel channel; the engine contact scan is
// kept only for the handlebar crash load it still reports. `final`
// selects the durable query like `sim.contact_query if final else
// self.probe_query`; handlebar_load is stateless either way.

RuntimeContacts
PhysicalStep::refresh_contacts(bool final, std::optional<double> time_s,
                               bool update_grounded, std::span<const double> qvel) {
    const mjModel *m = stepper_->model();
    const mjData *d = stepper_->data();
    RuntimeContactQuery &query =
        final ? contact_query_ : probe_query_;
    if (!stepper_->has_tire()) {
        auto result = query.query(m, d, time_s, qvel);
        const double t = time_s.value_or(d->time);
        const bool filtered = final || update_grounded;
        const bool front_loaded = working_load(result.snapshots[0]) > 1.;
        const bool rear_loaded = working_load(result.snapshots[1]) > 1.;
        result.contacts.front_controller_grounded =
            filtered ? filters_[0].update(front_loaded, t) : front_loaded;
        result.contacts.rear_controller_grounded =
            filtered ? filters_[1].update(rear_loaded, t) : rear_loaded;
        for (std::size_t i = 0; i < snapshots_.size(); ++i)
            snapshots_[i] = std::move(result.snapshots[i]);
        return result.contacts;
    }
    const double handlebar = query.handlebar_load(m, d);
    const TireWriterView tires = stepper_->tire_writer();
    const bool committed = tires.snapshots_committed();
    if (!committed) {
        // `if not snapshots` — committed snapshots stay empty until the
        // first advancing compute; the detached probe evaluates on a
        // cleared clock and only its snapshots cross back.
        tires.probe_compute_into(d, config_.timestep_s, component_out_);
    }
    // The engaged() contract the optional access carried: a probe that
    // returned without publishing leaves no snapshot to read.
    if (!committed && !tires.probe_committed())
        throw std::logic_error("engagement invariant violated");
    // self.snapshots — publish the committed pair (the detached probe's
    // when nothing has committed yet) into the durable member. The
    // slots mirror the oracle's publication lifecycle: disengaged from
    // construction/restore until this first publish engages them — and
    // the engagement reserves the committed slots' own construction
    // bound, so the copy-assign reuses capacity forever after instead
    // of materializing a fresh WheelSnapshots per refresh or regrowing
    // when the active patch count climbs past the first-published size
    // after a reset.
    const std::array<TireSnapshot, 2> &storage =
        committed ? tires.snapshot_storage()
                  : tires.probe_snapshot_storage();
    // The bound source is the committed storage — construction-reserved
    // on both writers regardless of the publication gate (tire.cpp:331,
    // distributed_tire.cpp:280-284); probe storage can carry less.
    const std::array<TireSnapshot, 2> &bound_slots =
        tires.snapshot_storage();
    for (std::size_t i = 0; i < 2; ++i) {
        if (!snapshots_[i].has_value()) {
            snapshots_[i].emplace();
            TireSnapshot &slot = engaged(snapshots_[i]);
            slot.patches.reserve(bound_slots[i].patches.capacity());
            slot.backend.reserve(bound_slots[i].backend.capacity());
        }
        snapshots_[i] = storage[i];
    }
    const double t = time_s.has_value() ? *time_s : d->time;
    const TireSnapshot &front = engaged(snapshots_[0]);
    const TireSnapshot &rear = engaged(snapshots_[1]);
    // grounded[side] = filters[side].update(loaded,t) if final or
    // update_grounded else loaded — the second observation of this
    // interval is idempotent at its equal timestamp.
    const bool filtered = final || update_grounded;
    const std::array<bool, 2> grounded{
        filtered ? filters_[0].update(working_load(front) > 1., t)
                 : working_load(front) > 1.,
        filtered ? filters_[1].update(working_load(rear) > 1., t)
                 : working_load(rear) > 1.};
    RuntimeContacts contacts;
    contacts.front_load_n = snapshot_normal_load(front);
    contacts.rear_load_n = snapshot_normal_load(rear);
    contacts.front_support_n = snapshot_normal_vertical(front);
    contacts.rear_support_n = snapshot_normal_vertical(rear);
    contacts.handlebar_load_n = handlebar;
    contacts.front_slip_mps = snapshot_slip(front);
    contacts.rear_slip_mps = snapshot_slip(rear);
    contacts.front_controller_grounded = grounded[0];
    contacts.rear_controller_grounded = grounded[1];
    return contacts;
}

// ---- _rollback_brake_demand (physical_runtime.py:196-213) ---------------

double PhysicalStep::rollback_brake_demand(double speed_mps,
                                         const RideControl *control) {
    const auto &drive_cfg = stepper_->drive().config();
    const auto &cfg = drive_cfg.policies.pedaling;
    if (!cfg.rollback_brake || cfg.rollback_demand <= 0.
        || (drive_cfg.drive_mode != "crank_effort"
            && drive_cfg.drive_mode != "articulated_effort")
        || (control != nullptr && !control->rider_enabled)) {
        rollback_hold_ = false;
        return 0.;
    }
    if (rollback_hold_)
        rollback_hold_ = speed_mps <= -cfg.rollback_release_mps;
    else
        rollback_hold_ = speed_mps < -cfg.rollback_engage_mps;
    return rollback_hold_ ? cfg.rollback_demand : 0.;
}

// ---- sensor_channels (physical_observations.py:170-186) ---------------
// Called after the constraint solve and before endpoint forward
// kinematics; encoder rows read the INCOMING velocity span.

WireObject
PhysicalStep::sensor_channels(std::span<const double> qvel,
                              const drivetrain::Diagnostics &last) const {
    const mjModel *m = stepper_->model();
    const mjData *d = stepper_->data();
    WireObject encoders;
    encoders.emplace_back("front_wheel",
                          Wire(qvel[static_cast<std::size_t>(
                              wheel_spin_dofs_[0])]));
    encoders.emplace_back("rear_wheel",
                          Wire(qvel[static_cast<std::size_t>(
                              wheel_spin_dofs_[1])]));
    encoders.emplace_back("crank",
                          Wire(qvel[static_cast<std::size_t>(
                              crank_spin_dof_)]));
    WireObject out;
    const auto sensordata = buf(d->sensordata, m->nsensordata);
    out.emplace_back("frame_specific_force_body_mps2",
                     wire_array(sensordata.subspan(
                         static_cast<std::size_t>(accel_sensor_adr_), 3)));
    out.emplace_back("frame_gyro_body_rad_s",
                     wire_array(sensordata.subspan(
                         static_cast<std::size_t>(gyro_sensor_adr_), 3)));
    out.emplace_back("encoders_rad_s", Wire(std::move(encoders)));
    out.emplace_back("motor_torque_nm",
                     Wire(diag_number(last, "motor_torque_nm")));
    out.emplace_back("human_torque_nm",
                     Wire(diag_number(last, "human_sensor_nm")));
    return out;
}

// ---- tire_channels (physical_observations.py:8-46) --------------------
// Called before solve: all Jacobian/velocity values belong to q_n, v_n.

WireObject PhysicalStep::tire_channels(const WheelSnapshots &snapshots,
                                       std::span<const double> qvel) const {
    const mjModel *m = stepper_->model();
    const mjData *d = stepper_->data();
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    std::array<std::optional<TireDiagnostics>, 2> diags;
    std::array<double, 2> radii{};
    bool distributed = false, pneumatic = false;
    if (stepper_->has_tire()) {
        const TireWriterView view = stepper_->tire_writer();
        diags = view.diagnostics();
        radii = view.radii();
        distributed = view.is_distributed();
        pneumatic = view.is_pneumatic();
    } else {
        const auto sizes = buf(m->geom_size, m->ngeom * 3);
        radii = {sizes[static_cast<std::size_t>(contact_query_.front_id) * 3],
                 sizes[static_cast<std::size_t>(contact_query_.rear_id) * 3]};
    }
    constexpr std::array<const char *, 2> sides = {"front", "rear"};
    const std::array<std::optional<bool>, 2> grounded = {
        contacts_.front_controller_grounded,
        contacts_.rear_controller_grounded};
    WireObject out;
    for (std::size_t i = 0; i < 2; ++i) {
        if (!snapshots[i].has_value())
            continue;
        const TireSnapshot &s = *snapshots[i];
        // runtime.drive.ids.get(side+'_wheel') — 'rear_wheel' resolves on
        // the elastic topology; 'front_wheel' always falls back to the
        // contact geom's body.
        const auto resolved =
            stepper_->drive().body_id(std::string(sides[i]) + "_wheel");
        const auto geom_body = ibuf(m->geom_bodyid, m->ngeom);
        const int body = resolved.value_or(
            geom_body[static_cast<std::size_t>(
                i == 0 ? contact_query_.front_id
                       : contact_query_.rear_id)]);
        const auto jac =
            point_jacobian(m, d, body, s.wheel_axis_m);
        const double omega =
            dot(std::span<const double>(jac.jr).subspan(nv, nv), qvel);
        const std::array<double, 3> velocity = {
            dot(std::span<const double>(jac.jp).subspan(0, nv), qvel),
            dot(std::span<const double>(jac.jp).subspan(nv, nv), qvel),
            dot(std::span<const double>(jac.jp).subspan(2 * nv, nv),
                qvel)};
        WireArray patches;
        patches.reserve(s.patches.size());
        for (const auto &p: s.patches) {
            WireObject row;
            row.emplace_back("point_m", wire_vec3(p.point_m));
            row.emplace_back("normal", wire_vec3(p.normal));
            row.emplace_back("normal_load_n", Wire(p.normal_load_n));
            row.emplace_back("tangent_force_n", Wire(p.tangent_force_n));
            row.emplace_back("slip_mps", Wire(p.slip_mps));
            row.emplace_back("world_force_n",
                             wire_vec3(patch_world_force(p)));
            row.emplace_back("couple_world_nm",
                             wire_vec3(p.couple_world_nm));
            row.emplace_back(
                "source_geom",
                Wire(std::string(p.working_surface ? "terrain"
                                                   : "catch_plane")));
            patches.emplace_back(std::move(row));
        }
        const double nl = snapshot_normal_load(s);
        NeumaierSum slip_num;
        for (const auto &p: s.patches)
            slip_num.add(p.slip_mps * p.normal_load_n);
        const double slip = nl > 0. ? slip_num.total() / nl : 0.;
        NeumaierSum roll_num;
        for (const auto &p: s.patches) {
            const std::array<double, 3> tangent{
                patch_tangent_component(p, 0),
                patch_tangent_component(p, 1),
                patch_tangent_component(p, 2)};
            roll_num.add(dot(velocity, tangent) * p.normal_load_n);
        }
        const double roll_speed =
            nl > 0. ? roll_num.total() / nl : velocity[0];
        // dict(diagnostics, ...) — the backend's own diagnostic key set
        // starts the row; observation keys then override/append in the
        // oracle's keyword order.
        WireObject side =
            diags[i].has_value()
                ? (distributed
                       ? wire_distributed_tire_diagnostics(*diags[i])
                       : pneumatic ? wire_pneumatic_tire_diagnostics(
                                 *diags[i])
                                   : wire_tire_diagnostics(*diags[i]))
                : WireObject{};
        const bool multi_support =
            diags[i].has_value() ? diags[i]->multi_support : false;
        const bool outside_load_range =
            diags[i].has_value() ? diags[i]->outside_material_load_range
                                 : false;
        const bool supports_multiple =
            diags[i].has_value() ? diags[i]->supports_multiple_contacts
                                 : false;
        wire_set(side, "backend", Wire(std::string(s.backend)));
        wire_set(side, "unloaded_radius_m", Wire(radii[i]));
        wire_set(side, "supports_multiple_contacts",
                 Wire(supports_multiple));
        wire_set(side, "outside_material_load_range",
                 Wire(outside_load_range));
        wire_set(side, "patches", Wire(std::move(patches)));
        wire_set(side, "geometric_contact", Wire(s.geometric_contact));
        wire_set(side, "raw_contact", Wire(snapshot_road_loaded(s)));
        wire_set(side, "normal_load_n", Wire(nl));
        wire_set(side, "world_force_n",
                 wire_vec3(snapshot_world_force(s)));
        wire_set(side, "vertical_force_n",
                 Wire(snapshot_world_force(s)[2]));
        wire_set(side, "normal_vertical_n",
                 Wire(snapshot_normal_vertical(s)));
        wire_set(side, "tangent_force_n",
                 Wire(snapshot_tangent_force(s)));
        wire_set(side, "effective_radius_m", Wire(s.effective_radius_m));
        wire_set(side, "wheel_axis_m", wire_vec3(s.wheel_axis_m));
        wire_set(side, "wheel_axis_moment_nm",
                 Wire(snapshot_axis_moment(s)));
        wire_set(side, "omega_abs_rad_s", Wire(omega));
        wire_set(side, "omega_rel_rad_s",
                 Wire(qvel[static_cast<std::size_t>(
                     wheel_spin_dofs_[i])]));
        wire_set(side, "slip_mps", Wire(slip));
        wire_set(side, "slip_ratio",
                 Wire(-slip / std::max(std::abs(roll_speed), .1)));
        wire_set(side, "tangent_center_speed_mps", Wire(roll_speed));
        wire_set(side, "controller_grounded",
                 Wire(grounded[i].value_or(false)));
        wire_set(side, "multi_support", Wire(multi_support));
        out.emplace_back(sides[i], Wire(std::move(side)));
    }
    return out;
}

// ---- stored_terms (physical_observations.py:79-100) -------------------
// Elastic storage terms evaluated at q, independently of the last force
// call. dict.update semantics merge the drive terms after the five
// suspension keys.

WireObject PhysicalStep::stored_terms() const {
    const mjModel *m = stepper_->model();
    const mjData *d = stepper_->data();
    const auto susp = stepper_->suspension().stored_terms(d);
    WireObject out;
    out.emplace_back("fork_air", Wire(susp.fork_air));
    out.emplace_back("shock_coil", Wire(susp.shock_coil));
    out.emplace_back("shock_bumper", Wire(susp.shock_bumper));
    out.emplace_back("shock_top_out", Wire(susp.shock_top_out));
    out.emplace_back("shock_upper_stop", Wire(susp.shock_upper_stop));
    for (const auto &[key, value]: stepper_->drive().stored_energy())
        wire_set(out, key, diag_wire(value));
    if (stepper_->has_tire())
        wire_set(out, "tires",
                 Wire(stepper_->tire_writer().stored_energy(d)));
    if (rider_control_.has_value())
        wire_set(out, "rider_joint_envelope",
                 Wire(rider_control_->envelope_forces(d).second));
    if (stepper_->has_rider_contacts())
        wire_set(out, "rider_interfaces",
                 Wire(stepper_->rider_contacts().stored_energy(d)));
    const auto qpos = buf(d->qpos, m->nq);
    if (stepper_->has_rider_forces())
        for (const auto &path: stepper_->rider_forces().paths()) {
            double depth = path.preload_deflection_m + path.offset_m -
                qpos[static_cast<std::size_t>(path.qposadr)];
            if (path.unilateral)
                depth = std::max(depth, 0.);
            wire_set(out, "seated_" + path.name,
                     Wire(.5 * path.stiffness_n_m * depth * depth));
        }
    // Explicit engine springs are stored separately from applied-force
    // springs ('engine_joint_<j>' in jnt index order).
    const auto jnt_stiff = buf(m->jnt_stiffness, m->njnt);
    const auto jnt_qposadr = ibuf(m->jnt_qposadr, m->njnt);
    const auto qpos_spring = buf(m->qpos_spring, m->nq);
    for (int j = 0; j < m->njnt; ++j) {
        const std::size_t u = static_cast<std::size_t>(j);
        if (jnt_stiff[u] == 0.)
            continue;
        const int qa = jnt_qposadr[u];
        const double q =
            qpos[static_cast<std::size_t>(qa)] -
            qpos_spring[static_cast<std::size_t>(qa)];
        wire_set(out, "engine_joint_" + std::to_string(j),
                 Wire(.5 * jnt_stiff[u] * q * q));
    }
    return out;
}

// ---- mass_observations (physical_energy.py:45-67) ---------------------
// Position/velocity caches are the caller's; mj_subtreeVel refreshes the
// subtree momentum observation rows without solving forces.

MassObservations PhysicalStep::mass_observations(mjModel *m, mjData *d) {
    const std::size_t nbody = static_cast<std::size_t>(m->nbody);
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    const auto body_mass = buf(m->body_mass, m->nbody);
    const auto xipos = buf(d->xipos, 3 * m->nbody);
    const double total = numpy_sum(body_mass);
    if (!(total > 0.))
        throw std::invalid_argument("model has no physical mass");
    const auto com = compiled_com(m, d);
    mul_scratch_.assign(nv, 0.);
    engine::mul_m(m, d, mul_scratch_.data(), d->qvel);
    const double kinetic =
        .5 * dot(std::span<const double>(
                     std::views::counted(d->qvel, m->nv)),
                 mul_scratch_);
    const auto grav = buf(std::data(m->opt.gravity), 3);
    std::vector<double> products(nbody * 3);
    for (std::size_t i = 0; i < nbody; ++i)
        for (std::size_t k = 0; k < 3; ++k)
            products[3 * i + k] =
                body_mass[i] * xipos[3 * i + k] * grav[k];
    const double gravity = -numpy_sum(products);
    engine::subtree_vel(m, d);
    const auto linvel = buf(d->subtree_linvel, 3);
    const auto angmom = buf(d->subtree_angmom, 3);
    std::array<double, 3> linear{}, velocity{};
    for (std::size_t k = 0; k < 3; ++k) {
        linear[k] = total * linvel[k];
        velocity[k] = linear[k] / total;
    }
    std::array<double, 3> angular{};
    for (std::size_t k = 0; k < 3; ++k)
        angular[k] = angmom[k];
    MassObservations out;
    out.kinetic_energy_j = kinetic;
    out.gravitational_energy_j = gravity;
    out.mass_kg = total;
    out.wire.emplace_back("mass_kg", Wire(total));
    out.wire.emplace_back("com_m", wire_vec3(com));
    out.wire.emplace_back("com_velocity_mps", wire_vec3(velocity));
    out.wire.emplace_back("linear_momentum_kg_mps", wire_vec3(linear));
    out.wire.emplace_back("angular_momentum_kg_m2_s",
                          wire_vec3(angular));
    out.wire.emplace_back("kinetic_energy_j", Wire(kinetic));
    out.wire.emplace_back("gravitational_energy_j", Wire(gravity));
    return out;
}

// ---- energy_state (physical_observations.py:102-105) ------------------

std::tuple<MassObservations, WireObject, double>
PhysicalStep::energy_state() {
    auto mass = mass_observations(stepper_->model(), stepper_->data());
    WireObject elastic = stored_terms();
    NeumaierSum elastic_sum;
    for (const auto &[key, value]: elastic)
        elastic_sum.add(wire_scalar(value));
    const double total = mass.kinetic_energy_j +
        mass.gravitational_energy_j + elastic_sum.total();
    return {std::move(mass), std::move(elastic), total};
}

// ---- _loss_increment (physical_runtime.py:512-545) --------------------
// Material losses only; road/aero/native-solver work stay signed
// external. The generator sum() is the Neumaier loop; every loss += that
// follows is a plain sequential float add like the oracle.

double PhysicalStep::loss_increment(
    const std::vector<std::pair<std::string, std::vector<double>>>
        &components,
    std::span<const double> velocity, double dt,
    const drivetrain::Diagnostics &last) const {
    NeumaierSum acc;
    for (const char *name:
         {"fork_damper", "shock_damper", "shock_hbo", "drive_bearings",
          "rider_passive_damping"})
        if (const auto *f = ordered_at(components, name))
            acc.add(std::max(0., -dot(*f, velocity)) * dt);
    double loss = acc.total();
    if (const auto *passive = ordered_at(components, "engine_passive")) {
        if (!last_force_sample_.has_value())
            throw std::runtime_error(
                "engine passive loss needs the interval-start sample");
        loss += engine_passive_loss_power(
                    stepper_->model(),
                    engaged(last_force_sample_).qpos, velocity,
                    *passive) *
            dt;
    }
    for (const char *name: {"front_static_brake", "rear_static_brake"}) {
        const auto *f = ordered_at(components, name);
        loss += std::max(0., -(f != nullptr ? dot(*f, velocity) : 0.)) *
            dt;
    }
    loss += diag_number(last, "chain_dissipation_power_w") * dt;
    loss += diag_number(last, "freehub_dissipation_power_w") * dt;
    loss += diag_number(last, "crank_clutch_dissipation_power_w") * dt;
    loss += diag_number(last, "motor_freewheel_dissipation_power_w") * dt;
    if (stepper_->has_tire())
        loss += stepper_->tire_writer().brush_loss_step_j() +
            stepper_->tire_writer().radial_dissipation_power_w() * dt;
    if (stepper_->has_rider_contacts()) {
        const auto &rc = stepper_->rider_contacts().state();
        loss += rc.loss_step_j + rc.radial_dissipation_power_w * dt;
    }
    if (stepper_->has_rider_forces()) {
        const auto paths = stepper_->rider_forces().paths();
        const auto telemetry = stepper_->rider_forces().path_telemetry();
        for (std::size_t i = 0; i < paths.size(); ++i) {
            const auto &path = paths[i];
            const double depth = path.preload_deflection_m + path.offset_m -
                engaged(last_force_sample_)
                    .qpos[static_cast<std::size_t>(path.qposadr)];
            const double elastic =
                path.stiffness_n_m * (path.unilateral ? std::max(depth, 0.)
                                                      : depth);
            loss += std::max(0.,
                             -(telemetry[i].force_n - elastic) *
                                 velocity[static_cast<std::size_t>(
                                     path.dofadr)]) *
                dt;
        }
    }
    // End-stop damping is superimposed on an explicitly integrated
    // spring.
    const auto &susp = stepper_->suspension();
    const auto &scfg = susp.config();
    const double x =
        engaged(last_force_sample_)
            .qpos[static_cast<std::size_t>(susp.shock_qposadr())];
    const double shock_v = velocity[static_cast<std::size_t>(
        susp.shock_dofadr())];
    const double nominal = scfg.coil.stroke_mm / 1000.;
    if (x < 0.) {
        const double elastic = -scfg.end_stops.stiffness_n_m * x;
        loss += std::max(0.,
                         -(require_component(components, "shock_top_out")
                                   [static_cast<std::size_t>(
                                       susp.shock_dofadr())] -
                           elastic) *
                             shock_v) *
            dt;
    }
    if (x > nominal) {
        const double elastic =
            -(scfg.coil.bumper_peak_n +
              scfg.end_stops.stiffness_n_m * (x - nominal));
        loss += std::max(0.,
                         -(require_component(components,
                                             "shock_upper_stop")
                                   [static_cast<std::size_t>(
                                       susp.shock_dofadr())] -
                           elastic) *
                             shock_v) *
            dt;
    }
    return loss;
}

// ---- _balance_channel (physical_runtime.py:547-551) -------------------
// The monitor update rides inside the channel emission exactly like the
// oracle; `riding` reads the post-hold demands and resolved control.

WireObject PhysicalStep::balance_channel(double front, double rear,
                                       bool braking,
                                       bool rider_enabled) {
    const mjModel *m = stepper_->model();
    const mjData *d = stepper_->data();
    balance_monitor_.update(
        d->time,
        buf(d->qpos, m->nq)[static_cast<std::size_t>(root_x_qposadr_)],
        buf(d->qvel, m->nv)[static_cast<std::size_t>(root_x_dofadr_)],
        rider_control_.has_value() && rider_enabled && front < .5 &&
            rear < .5 && !braking);
    return balance_monitor_.observation();
}

// ---- _update_compiled_com_marker (ride_sim.py:429-440) -----------------
// site_pos[cg] = xmat[frame].T @ (com - xpos[frame]) — a model write the
// mutate frame owns; site_xpos[cg] mirrors the world CoM.

void PhysicalStep::update_compiled_com_marker() {
    if (cg_site_id_ < 0)
        return;
    // site_pos/site_xpos writes go through the member pointers — the
    // model/data handles themselves stay const like the read path.
    const mjModel *m = stepper_->model();
    const mjData *d = stepper_->data();
    const auto com = compiled_com(m, d);
    const auto xpos = buf(d->xpos, 3 * m->nbody);
    const std::size_t fb = static_cast<std::size_t>(frame_body_);
    const std::array<double, 3> diff{
        com[0] - xpos[3 * fb],
        com[1] - xpos[3 * fb + 1],
        com[2] - xpos[3 * fb + 2]};
    const auto xmat = buf(d->xmat, 9 * m->nbody).subspan(9 * fb);
    blas::dgemv(blas::Order::row_major, blas::Transpose::yes, 3, 3, 1.,
                xmat.data(), 3, diff.data(), 1, 0.,
                wbuf(m->site_pos, 3 * m->nsite)
                    .subspan(3 * static_cast<std::size_t>(cg_site_id_))
                    .data(),
                1);
    const auto site_x = wbuf(d->site_xpos, 3 * m->nsite);
    for (std::size_t k = 0; k < 3; ++k)
        site_x[3 * static_cast<std::size_t>(cg_site_id_) + k] = com[k];
}

// ---- update_rider_intent_signals (physical_runtime.py:454-459) --------
// signals_from_channels on the channel dict the same interval published.

void PhysicalStep::update_intent_signals(
    const WireObject &sensors_and_tires) {
    const WireObject *tires = wire_obj(sensors_and_tires, "tires");
    double front = 0., rear = 0.;
    if (tires != nullptr) {
        if (const auto *f = wire_obj(*tires, "front"))
            front = wire_num(*f, "normal_load_n");
        if (const auto *r = wire_obj(*tires, "rear"))
            rear = wire_num(*r, "normal_load_n");
    }
    const double total = front + rear;
    rider::IntentSignals s;
    s.pitch_rate_up_rad_s =
        -wire_arr_at(sensors_and_tires, "frame_gyro_body_rad_s", 1);
    for (std::size_t k = 0; k < 3; ++k)
        s.specific_force_body_mps2[k] =
            wire_arr_at(sensors_and_tires,
                        "frame_specific_force_body_mps2", k);
    if (const auto *enc = wire_obj(sensors_and_tires, "encoders_rad_s"))
        s.crank_rate_rad_s = wire_num(*enc, "crank");
    s.human_crank_torque_nm =
        wire_num(sensors_and_tires, "human_torque_nm");
    if (total > 1.)
        s.front_load_share = front / total;
    else
        s.front_load_share = std::nullopt;
    intent_signals_ = s;
}

// ---- probe_step_inputs (apply_forces advance=false) --------------------
// The staged-input probe: the same ordered force fold, actuator input
// row, model brake bounds and staged diagnostics views an advancing
// interval's pre-solve assembly leaves behind — without committing any
// state (probe publication rules apply per writer).

PhysicalStep::ProbeResult
PhysicalStep::probe_step_inputs(const RideControl &control,
                                double front_demand,
                                double rear_demand) {
    return stepper_->mutate([&]() -> ProbeResult {
        control_validate_for(control, rider_control_.has_value(), drive_mode());
        scalar_field(front_demand, "front brake demand");
        scalar_field(rear_demand, "rear brake demand");
        const mjModel *m = stepper_->model();
        const mjData *d = stepper_->data();
        (void)apply_forces(true, false, front_demand, rear_demand,
                           std::nullopt, control, std::nullopt);
        ProbeResult out;
        out.components = accumulator_.components();
        out.ctrl = buffer_copy(d->ctrl, m->nu);
        const auto frictionloss = buf(m->dof_frictionloss, m->nv);
        out.front_brake_bound_nm =
            frictionloss[static_cast<std::size_t>(brake_.front_dof())];
        out.rear_brake_bound_nm =
            frictionloss[static_cast<std::size_t>(brake_.rear_dof())];
        out.drive = std::get<WireObject>(std::move(
            wire_diagnostics(
                stepper_->drive().diagnostics(true).to_map())
                .value));
        if (rider_control_.has_value()) {
            const auto &probe = stepper_->rider_contacts().probe();
            if (probe.has_value()) {
                out.has_contact_probe = true;
                out.contact_probe =
                    wire_contacts_diagnostics(probe->diagnostics);
            }
        }
        // Observability lanes: the channel dict sensor_channels() emits
        // for the committed data (encoders read the incoming velocity
        // span — the current qvel at probe time), plus the committed
        // rider-intent signals and policy inclination.
        out.sensors = sensor_channels(
            buf(d->qvel, m->nv),
            stepper_->drive().diagnostics(false).to_map());
        WireObject signals;
        signals.emplace_back("pitch_rate_up_rad_s",
                             Wire(intent_signals_.pitch_rate_up_rad_s));
        signals.emplace_back(
            "specific_force_body_mps2",
            wire_array(std::span<const double>(
                intent_signals_.specific_force_body_mps2)));
        signals.emplace_back("crank_rate_rad_s",
                             Wire(intent_signals_.crank_rate_rad_s));
        signals.emplace_back("human_crank_torque_nm",
                             Wire(intent_signals_.human_crank_torque_nm));
        signals.emplace_back(
            "front_load_share",
            intent_signals_.front_load_share.has_value()
                ? Wire(*intent_signals_.front_load_share)
                : Wire(nullptr));
        out.intent_signals = std::move(signals);
        out.intent_inclination_rad =
            rider_intent_.policy().inclination_rad();
        return out;
    });
}

// ---- state round-trip --------------------------------------------------
// state() reads live owners; restore() stages then commits every member.
// The binding validates the wire shape BEFORE staging, so a rejected
// field never half-commits.

StepState PhysicalStep::state() const {
    StepState s;
    s.step = step_;
    s.generation = generation_;
    s.measurement_decimation = measurement_decimation_;
    s.applied_control = applied_control_;
    s.held_control = control_clock_.held_state();
    s.held_rider_terms = held_rider_terms_;
    s.rollback_hold = rollback_hold_;
    s.research_accounting_valid = research_accounting_valid_;
    s.initializing = initializing_;
    s.filters = {filters_[0].state(), filters_[1].state()};
    s.balance = balance_monitor_.state();
    s.crash = crash_detector_.event();
    s.contacts = contacts_;
    s.contact_query = contact_query_.state();
    s.probe_query = probe_query_.state();
    s.energy = energy_;
    s.signals = intent_signals_;
    if (rider_control_.has_value())
        s.rider_controller = rider_control_->state();
    s.rider_intent = rider_intent_.state();
    return s;
}

void PhysicalStep::restore(const StepState &s) {
    step_ = s.step;
    generation_ = s.generation;
    measurement_decimation_ = s.measurement_decimation;
    applied_control_ = s.applied_control;
    if (s.held_control.has_value())
        control_clock_.hold(*s.held_control);
    else
        control_clock_.reset();
    held_rider_terms_ = s.held_rider_terms;
    rollback_hold_ = s.rollback_hold;
    research_accounting_valid_ = s.research_accounting_valid;
    initializing_ = s.initializing;
    filters_[0].restore(s.filters[0]);
    filters_[1].restore(s.filters[1]);
    balance_monitor_.restore(s.balance);
    crash_detector_.set_event(s.crash);
    contacts_ = s.contacts;
    contact_query_.restore(s.contact_query);
    probe_query_.restore(s.probe_query);
    energy_ = s.energy;
    intent_signals_ = s.signals;
    if (s.rider_controller.has_value() && rider_control_.has_value())
        rider_control_->restore(*s.rider_controller);
    rider_intent_.restore(s.rider_intent);
    // The interval-local captures are not wire-serialized (setup.py's
    // state inventory omits them): they repopulate on the first step
    // exactly like the oracle's first interval does.
    last_force_sample_.reset();
    last_constraint_snapshot_ = ConstraintSnapshot{};
    snapshots_ = WheelSnapshots{};
    rider_state_.reset();
}

} // namespace runtime
