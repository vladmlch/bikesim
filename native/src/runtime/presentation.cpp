// Physical HUD/preview capture. Source references supplied with this delivery:
// NumPy numpy/_core/src/npymath/npy_math_internal.h.src, npy_rad2deg:
//     x * (180.0 / NPY_PI), not (x * 180.0) / pi.
// MuJoCo mjtype.h/mujoco.h: owning mjSTATE_INTEGRATION and model field replicas.
#include "presentation.hpp"
#include "step.hpp"
#include "../model_access.hpp"
#include "../stepper.hpp"
#include "../drivetrain/pedaling.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/suspension.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <variant>

namespace runtime {
namespace {
constexpr std::array<const char *, 6> kModelFields = {
    "dof_frictionloss", "site_pos", "tendon_stiffness", "tendon_damping",
    "tendon_lengthspring", "tendon_range"};
constexpr double degrees(double value) noexcept {
    return value * (180.0 / std::numbers::pi);
}
std::size_t extent(mjtSize count, std::size_t width) {
    if (count < 0) throw std::invalid_argument("presentation: negative model extent");
    return model_access::checked_product(static_cast<std::size_t>(count), width,
                                         static_cast<std::size_t>(std::numeric_limits<mjtSize>::max()));
}
std::span<const double> array(const double *pointer, mjtSize count, std::size_t width = 1) {
    return model_access::readonly_buffer(pointer, static_cast<mjtSize>(extent(count, width)),
                                         "presentation model field");
}
std::array<std::span<const double>, 6> model_fields(const mjModel *model) {
    return {array(model->dof_frictionloss, model->nv), array(model->site_pos, model->nsite, 3),
            array(model->tendon_stiffness, model->ntendon), array(model->tendon_damping, model->ntendon),
            array(model->tendon_lengthspring, model->ntendon, 2), array(model->tendon_range, model->ntendon, 2)};
}
int joint_qpos(const mjModel *model, const char *name) {
    const int joint = model_access::resolve_id(model, mjOBJ_JOINT, name);
    const auto addresses = model_access::readonly_buffer(model->jnt_qposadr, model->njnt);
    const int address = addresses[static_cast<std::size_t>(joint)];
    model_access::require_id(address, model->nq, "presentation joint qpos");
    return address;
}
double body_pitch(const mjModel *model, const mjData *data, const char *name) {
    const int body = model_access::resolve_id(model, mjOBJ_BODY, name);
    const auto rotations = array(data->xmat, model->nbody, 9);
    const std::size_t offset = static_cast<std::size_t>(body) * 9U;
    return degrees(std::atan2(-rotations[offset + 6U], rotations[offset]));
}
Wire diagnostic(const drivetrain::Diagnostics &drive, const char *name, Wire fallback = Wire{0.}) {
    const auto found = drive.find(name);
    if (found == drive.end()) return fallback;
    return std::visit([](const auto &value) -> Wire {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::monostate>) return Wire{nullptr};
        else if constexpr (std::is_same_v<T, int>) return Wire{static_cast<std::int64_t>(value)};
        else return Wire{value};
    }, found->second);
}
double number(const drivetrain::Diagnostics &drive, const char *name, double fallback = 0.) {
    const auto value = diagnostic(drive, name, Wire{fallback});
    if (const auto *v = std::get_if<double>(&value.value)) return *v;
    if (const auto *v = std::get_if<std::int64_t>(&value.value)) return static_cast<double>(*v);
    if (const auto *v = std::get_if<bool>(&value.value)) return *v ? 1. : 0.;
    return fallback;
}
Wire vector_wire(std::span<const double> values) {
    WireArray out;
    out.reserve(values.size());
    for (const double value : values) out.emplace_back(value);
    return Wire{std::move(out)};
}
void append_name(std::string &out, const std::string &name) {
    if (!out.empty()) out += ',';
    out += name;
}
} // namespace

void PhysicalStep::initialize_presentation(PresentationState &out) const {
    out.rider_present = rider_control_.has_value();
    out.tire_present = stepper_->tire() != nullptr;
    const mjModel *const model = stepper_->model();
    const auto fields = model_fields(model);
    for (std::size_t i = 0; i < fields.size(); ++i) out.model_fields[i].resize(fields[i].size());
    // Match the writer's bounded open-label capacities before the first tick.
    out.drive.ensure_open_capacity(drivetrain::TelemetryField::coasting_reason,
        drivetrain::coast_reason_name(drivetrain::CoastReason::no_effort).size());
    out.drive.ensure_open_capacity(drivetrain::TelemetryField::assist_mode,
        stepper_->drive().config().policies.assist.mode.size());
    if (rider_control_) {
        if (model->njnt > std::numeric_limits<int>::max())
            throw std::invalid_argument("presentation: joint count exceeds the MuJoCo ID range");
        std::size_t length = 0;
        for (int index = 0; static_cast<mjtSize>(index) < model->njnt; ++index) {
            const char *name = mj_id2name(model, mjOBJ_JOINT, index);
            if (name != nullptr && std::string_view(name).starts_with("rider_")) {
                out.joints.emplace_back(name, 0.);
                out.joint_qpos_addresses.push_back(joint_qpos(model, name));
                length += std::string_view(name).size() + 1U;
            }
        }
        out.joints_saturated.reserve(length);
        out.ik_saturated.reserve(32);
    }
    capture_presentation(out);
}

void PhysicalStep::capture_presentation(PresentationState &out) const {
    const mjModel *const model = stepper_->model();
    const mjData *const data = stepper_->data();
    const auto q = stepper_->qpos();
    const auto v = stepper_->qvel();
    const auto &suspension = stepper_->suspension();
    out.time_s = data->time;
    out.position_m = q[static_cast<std::size_t>(root_x_qposadr_)];
    out.speed_mps = v[static_cast<std::size_t>(root_x_dofadr_)];
    out.root_x_dofadr = root_x_dofadr_;
    out.pitch_rad = q[static_cast<std::size_t>(root_pitch_qposadr_)];
    model_access::require_id(frame_body_, model->nbody, "presentation frame");
    out.z_m = array(data->xpos, model->nbody, 3)[static_cast<std::size_t>(frame_body_) * 3U + 2U];
    out.crank_phase_rad = q[static_cast<std::size_t>(crank_spin_qposadr_)];
    out.fork_travel_mm = q[static_cast<std::size_t>(suspension.fork_qposadr())] * 1000.;
    out.shock_stroke_mm = q[static_cast<std::size_t>(suspension.shock_qposadr())] * 1000.;
    out.fork_velocity_mps = v[static_cast<std::size_t>(suspension.fork_dofadr())];
    out.shock_velocity_mps = v[static_cast<std::size_t>(suspension.shock_dofadr())];
    const auto *fork = accumulator_.component("fork_spring");
    const auto *shock = accumulator_.component("shock_coil");
    out.fork_force_n = fork == nullptr ? 0. : fork->at(static_cast<std::size_t>(suspension.fork_dofadr()));
    out.shock_force_n = shock == nullptr ? 0. : shock->at(static_cast<std::size_t>(suspension.shock_dofadr()));
    out.front_load_n = contacts_.front_load_n;
    out.rear_load_n = contacts_.rear_load_n;
    out.drive = stepper_->drive().diagnostics(false);
    out.assist_pedaling = stepper_->drive().assist_pedaling();
    out.battery_energy_j = stepper_->drive().battery_energy_j();
    const auto balance = balance_event();
    out.balance_lost_at_m = balance ? std::optional<double>(balance->position_m) : std::nullopt;
    // rider_present was initialized from rider_control_ and cannot diverge.
    if (rider_control_) {
        const auto &diagnostics = stepper_->rider_contacts().state().diagnostics;
        out.grip_enabled = diagnostics.grip.enabled;
        out.grip_reachable = diagnostics.grip.reachable;
        out.grip_gap_m = diagnostics.grip.hand_gap_m;
        for (std::size_t i = 0; i < out.supports.size(); ++i) {
            const auto &source = diagnostics.supports[i];
            out.supports[i] = PresentationSupport{.in_platform = source.in_platform,
                .normal_load_n = source.normal_load_n, .gap_m = source.gap_m};
        }
        const auto &support = rider_control_->presentation_support();
        out.stance_front = support.stance_front;
        out.stance_rear = support.stance_rear;
        out.ik_saturated.clear();
        for (const auto &[name, active] : rider_control_->presentation_ik())
            if (active) append_name(out.ik_saturated, name);
        capture_accounted_presentation(out);
        out.root_pitch_deg = degrees(q[static_cast<std::size_t>(joint_qpos(model, "rider_root_pitch"))]);
        out.pelvis_pitch_deg = body_pitch(model, data, "rider_pelvis");
        out.torso_pitch_deg = body_pitch(model, data, "rider_torso");
        for (std::size_t i = 0; i < out.joints.size(); ++i)
            out.joints[i].second = q[static_cast<std::size_t>(out.joint_qpos_addresses[i])];
    }
    out.tire_slip = {};
    out.tire_mu = {};
    if (out.tire_present && stepper_->tire_writer().snapshots_committed()) {
        const auto &tires = stepper_->tire_writer().diagnostic_storage();
        for (std::size_t i = 0; i < tires.size(); ++i) {
            out.tire_slip[i] = tires[i].slip_mps;
            out.tire_mu[i] = tires[i].friction_coefficient;
        }
    }
    const auto fields = model_fields(model);
    for (std::size_t i = 0; i < fields.size(); ++i)
        std::ranges::copy(fields[i], out.model_fields[i].begin());
}

void PhysicalStep::capture_accounted_presentation(PresentationState &out) const {
    // This reads only accounting-published controller diagnostics. It remains
    // safe after a fatal engine call; no mjData or force evaluation is used.
    out.joints_saturated.clear();
    if (rider_control_)
        for (const auto &[name, terms] : rider_control_->presentation_terms())
            if (terms.saturated) append_name(out.joints_saturated, name);
}

WireObject PresentationState::as_wire() const {
    const auto last = drive.to_map();
    const auto set = [](WireObject &object, std::string key, Wire value) {
        sample_wire::set(object, std::move(key), std::move(value));
    };
    WireObject row{{"time_s", time_s}, {"x_m", position_m}, {"rtf", nullptr},
        {"speed_kmh", speed_mps * 3.6}, {"pitch_deg", degrees(pitch_rad)},
        {"grade_pct", 0.}, {"obstacle", ""}, {"obstacle_distance_m", ""},
        {"front_load_n", front_load_n}, {"rear_load_n", rear_load_n},
        {"fork_travel_mm", fork_travel_mm}, {"fork_velocity_mps", fork_velocity_mps},
        {"fork_force_n", fork_force_n}, {"shock_stroke_mm", shock_stroke_mm},
        {"shock_velocity_mps", shock_velocity_mps}, {"shock_force_n", shock_force_n},
        {"crank_phase_rad", crank_phase_rad}};
    const Wire phase = diagnostic(last, "crank_target_phase_rad", Wire{nullptr});
    const Wire rate = diagnostic(last, "crank_target_rate_rad_s", Wire{nullptr});
    set(row, "crank_target_rate_rpm", sample_wire::is_none(phase) || sample_wire::is_none(rate)
        ? Wire{""} : Wire{number(last, "crank_target_rate_rad_s") * 60. / (2. * std::numbers::pi)});
    for (const char *key : {"cadence_rpm", "required_cadence_rpm", "gear_front_teeth", "gear_rear_teeth",
            "shift_count", "freehub_torque_nm", "human_sensor_nm", "human_command_nm", "assist_sensor_nm",
            "motor_request_nm", "motor_torque_nm", "motor_shaft_power_w"})
        set(row, key, diagnostic(last, key));
    set(row, "rider_mode", diagnostic(last, "rider_mode", Wire{"unknown"}));
    const Wire coast = diagnostic(last, "coasting_reason", Wire{""});
    set(row, "coast_reason", sample_wire::is_none(coast) ? Wire{""} : coast);
    set(row, "shift_torque_factor", diagnostic(last, "shift_torque_factor", Wire{1.}));
    const bool shifted = number(last, "shift_count") != 0.;
    for (const auto &[destination, source] : {
            std::pair{"last_shift_direction", "shift_direction"},
            std::pair{"last_shift_from_teeth", "shift_from_teeth"},
            std::pair{"last_shift_to_teeth", "gear_rear_teeth"}})
        set(row, destination, shifted ? diagnostic(last, source, Wire{""}) : Wire{""});
    set(row, "last_shift_time_s", shifted ? diagnostic(last, "shift_time_s") : Wire{""});
    set(row, "freehub_engaged", Wire{static_cast<std::int64_t>(number(last, "freehub_engaged") != 0.)});
    set(row, "motor_enabled", Wire{static_cast<std::int64_t>(number(last, "motor_enabled") != 0.)});
    set(row, "assist_pedaling", Wire{static_cast<std::int64_t>(assist_pedaling)});
    for (const char *key : {"rider_grip_enabled", "rider_grip_reachable", "rider_pedal_front", "rider_pedal_rear",
            "rider_ik_saturated", "rider_stance_front", "rider_stance_rear", "grip_gap_m", "front_pedal_load_n",
            "front_pedal_gap_m", "rear_pedal_load_n", "rear_pedal_gap_m", "saddle_load_n", "rider_root_pitch_deg",
            "rider_pelvis_pitch_deg", "rider_torso_pitch_deg", "rider_rel_pitch_deg", "rider_joints_saturated"})
        set(row, key, Wire{""});
    if (rider_present) {
        set(row, "rider_grip_enabled", Wire{static_cast<std::int64_t>(grip_enabled)});
        set(row, "rider_grip_reachable", Wire{static_cast<std::int64_t>(grip_reachable)});
        set(row, "rider_pedal_front", Wire{static_cast<std::int64_t>(supports[1].in_platform)});
        set(row, "rider_pedal_rear", Wire{static_cast<std::int64_t>(supports[2].in_platform)});
        set(row, "rider_ik_saturated", Wire{ik_saturated});
        set(row, "rider_joints_saturated", Wire{joints_saturated});
        set(row, "rider_stance_front", Wire{static_cast<std::int64_t>(stance_front)});
        set(row, "rider_stance_rear", Wire{static_cast<std::int64_t>(stance_rear)});
        set(row, "grip_gap_m", Wire{grip_gap_m});
        set(row, "front_pedal_load_n", Wire{supports[1].normal_load_n});
        set(row, "front_pedal_gap_m", Wire{supports[1].gap_m});
        set(row, "rear_pedal_load_n", Wire{supports[2].normal_load_n});
        set(row, "rear_pedal_gap_m", Wire{supports[2].gap_m});
        set(row, "saddle_load_n", Wire{supports[0].normal_load_n});
        set(row, "rider_root_pitch_deg", Wire{root_pitch_deg});
        set(row, "rider_pelvis_pitch_deg", Wire{pelvis_pitch_deg});
        set(row, "rider_torso_pitch_deg", Wire{torso_pitch_deg});
        set(row, "rider_rel_pitch_deg", Wire{root_pitch_deg - degrees(pitch_rad)});
    }
    for (std::size_t i = 0; i < 2; ++i) {
        const std::string side = i == 0 ? "front" : "rear";
        set(row, side + "_slip_mps", tire_present ? Wire{tire_slip[i]} : Wire{""});
        set(row, side + "_mu", tire_present ? Wire{tire_mu[i]} : Wire{""});
    }
    WireObject angles;
    for (const auto &[name, angle] : joints) angles.emplace_back(name, Wire{angle});
    WireObject endpoint{{"time_s", time_s}, {"position_m", position_m}, {"speed_mps", speed_mps},
        {"pitch_rad", pitch_rad}, {"z_m", z_m}, {"root_x_dofadr", static_cast<std::int64_t>(root_x_dofadr)},
        {"torso_pitch_deg", rider_present ? Wire{torso_pitch_deg} : Wire{nullptr}},
        {"rider_joint_angles_rad", Wire{std::move(angles)}}, {"battery_energy_j", battery_energy_j},
        {"balance_lost_at_m", balance_lost_at_m ? Wire{*balance_lost_at_m} : Wire{nullptr}}};
    WireObject fields;
    for (std::size_t i = 0; i < model_fields.size(); ++i)
        fields.emplace_back(kModelFields[i], vector_wire(model_fields[i]));
    return {{"drive_mode", "articulated_effort"}, {"endpoint", Wire{std::move(endpoint)}},
            {"preview_row", Wire{std::move(row)}}, {"model_fields", Wire{std::move(fields)}}};
}
} // namespace runtime
