// rider/spindle_controller.hpp — the spindle-branch port of
// ArticulatedRiderController (rider_control.py) for the supported welded
// configuration (pedal_attachment='spindle', grip_attachment='connect',
// saddle_attachment='pin').
//
// Only the spindle code path is implemented: two-leg waveform -> closed
// leg-loop Jacobians -> directional crank split -> upper-body PD and bias
// compensation -> directional/speed/power/strength limits -> activation ->
// finalize_effort bookkeeping. The legacy QP allocator (_compute_legacy)
// does not exist in the supported configuration and is not ported; the
// runtime rejects configurations that would reach it.
//
// Every method mirrors the Python statements it names; the Python ordering
// fixes the FP contract. The class is mjModel/mjData-facing only — no
// nanobind types enter this header; dict decoding lives in the binding
// translation units.
#pragma once
#include <mujoco/mujoco.h>

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "rider_posture.hpp"
#include "spindle_math.hpp"

namespace rider {

// pedal_recovery.py PedalRecovery — state fields only. The observe/goal
// state machine is reached only by the unsupported flat-pedal path; the
// spindle controller carries these fields because reset_activation and the
// bootstrap wire do.
struct PedalRecoveryState {
    std::string stage = "none";
    double direction = 1.;
    double release_offset_x_m = 0.;
};

// The ArticulatedPose fields the spindle path reads, as fixed 3-vectors.
struct SpindlePose {
    std::array<double, 3> hip{};
    std::array<double, 3> shoulder{};
    std::array<double, 3> elbow{};
    std::array<double, 3> grip{};
    std::array<double, 3> knee_front{};
    std::array<double, 3> knee_rear{};
    std::array<double, 3> pedal_front{};
    std::array<double, 3> pedal_rear{};
};

struct DirectionalStrength {
    spindle::TorqueCurve positive;
    spindle::TorqueCurve negative;
};

// The spindle-usable subset of ArticulatedConfig plus the resolved tables
// the setup boundary appends beside the asdict fields. Path strings stay
// informational — the parsed tables are what the controller consumes.
struct SpindleConfig {
    double joint_kp_nm_rad = 0.;
    double joint_kd_nms_rad = 0.;
    double joint_limit_nm = 0.;
    double joint_speed_limit_rad_s = 0.;
    double joint_power_limit_w = 0.;
    std::optional<double> active_positive_power_limit_w;
    std::optional<double> joint_passive_damping_nms_rad;
    double activation_tau_s = 0.;
    double pedal_torque_ripple = 0.;
    double return_foot_preload_n = 0.;
    double coasting_brake_d_nm_s_rad = 0.;
    double coasting_brake_limit_nm = 0.;
    double joint_envelope_soft_k_nm_rad = 0.;
    double joint_envelope_soft_margin_rad = 0.;
    double arm_reach_fraction = 0.;
    double pedal_patch_half_length_m = 0.;
    double support_pad_radius_m = 0.;
    bool has_strength = false;
    std::map<std::string, DirectionalStrength> strength;
    std::map<std::string, spindle::JointEnvelope> strength_coordinates;
    std::map<std::string, spindle::JointEnvelope> envelopes;

    [[nodiscard]] double passive_damping_nms_rad() const {
        return joint_passive_damping_nms_rad.value_or(joint_kd_nms_rad);
    }
};

// rider_control.py RiderCommand.
struct RiderCommand {
    double mean_crank_torque_nm = 0.;
    bool enabled = true;
    RiderPosture posture;
    std::optional<double> crank_target_phase_rad;
    double crank_target_rate_rad_s = 0.;
};

// Insertion-ordered name->T pairs — mirrors Python dict ordering on the
// wire (alphabetical std::map order would reorder the anatomical joint
// sequence). Linear lookup is fine at nine joints.
template <typename T>
using NamedEntries = std::vector<std::pair<std::string, T>>;

// last_terms entry — base keys from compute plus the finalize_effort and
// solved_effort extension keys (present flags govern emission).
struct JointTerms {
    double requested_nm = 0.;
    double command_nm = 0.;
    double posture_nm = 0.;
    double pedaling_nm = 0.;
    bool saturated = false;
    std::optional<double> active_request_nm;
    std::optional<double> active_delivered_nm;
    std::optional<double> passive_damping_nm;
    std::optional<double> solved_force_nm;
    std::optional<double> solved_active_nm;
    std::optional<double> solved_passive_nm;
};

// effort_diagnostics — the finalize_effort key set plus the solved_effort
// additions (each optional field emitted only when solved_effort ran).
struct EffortDiagnostics {
    bool present = false;
    NamedEntries<double> active_request_nm;
    NamedEntries<double> active_delivered_nm;
    double positive_power_w = 0.;
    double passive_power_w = 0.;
    bool activation_saturated = false;
    std::vector<std::string> strength_limited;
    bool budget_exceeded = false;
    std::optional<double> positive_power_limit_w;
    std::string observation;
    std::optional<NamedEntries<double>> joint_positive_power_w;
    std::optional<std::vector<std::string>> joint_power_violations;
    std::optional<std::vector<std::string>> joint_speed_violations;
    std::optional<double> positive_work_step_j;
    std::optional<double> passive_work_step_j;
    std::optional<std::vector<std::string>> strength_violations;
};

struct AllocationDiagnostics {
    bool present = false;
    bool invalid_controller = false;
    double crank_task_nm = 0.;
    double crank_task_shortfall_nm = 0.;
    std::vector<double> solution_excitation_nm;
};

struct SupportDiagnostics {
    bool present = false;
    bool stance_front = false;
    bool stance_rear = false;
};

struct SoleGoalDiagnostics {
    bool spindle = true;
    bool saturated = false;
    std::vector<std::string> limiting_reasons;
};

// lean_limit.py: forward-lean ceiling over pose + anatomical envelopes.
[[nodiscard]] double lean_limit_rad(
    const SpindlePose &pose,
    const std::map<std::string, spindle::JointEnvelope> &envelopes,
    double crank_m, double tdc_phase_rad, double max_rad = .8,
    double tolerance = 1e-3);

class SpindleController {
public:
    struct JointEntry {
        std::string name;
        int qpos_adr;
        int dof_adr;
        int actuator_id;
    };

    // resolve + geometry once — mirrors __init__'s spindle branch, including
    // the IK neutral, grip-connect offsets, envelope sync and the optional
    // lean-limit solve. Throws std::invalid_argument on any missing or
    // malformed topology.
    SpindleController(const mjModel *model, const SpindlePose &pose,
                      SpindleConfig config, double crank_length_m);
    ~SpindleController();

    SpindleController(const SpindleController &) = delete;
    SpindleController &operator=(const SpindleController &) = delete;
    SpindleController(SpindleController &&) = delete;
    SpindleController &operator=(SpindleController &&) = delete;

    // compute() spindle branch (rider_control.py:756-843). Returns the
    // torque dict in joint order like the oracle. dt_s falls back to
    // model.opt.timestep exactly like Python's None default.
    [[nodiscard]] NamedEntries<double>
    compute(mjData *data, const RiderCommand &command, bool advance = true,
            std::optional<double> dt_s = std::nullopt,
            bool steady_state = false);

    // initialize()/initialize_velocity() — static-equilibrium-time pose
    // matching only; never inside the running step.
    void initialize(mjData *data);
    void initialize_velocity(mjData *data);

    void reset_activation();

    // envelope_forces — (nv torque vector, summed stored energy).
    [[nodiscard]] std::pair<std::vector<double>, double>
    envelope_forces(const mjData *data) const;

    // write() — torques -> data.ctrl through actuator ids.
    void write(mjData *data, const NamedEntries<double> &torques) const;

    // strength surfaces — directional_capacity per joint.
    [[nodiscard]] double strength_capacity(const std::string &name,
                                           double angle_rad,
                                           double velocity_rad_s,
                                           double torque_nm) const;
    [[nodiscard]] double anatomical_joint_angle(const std::string &name,
                                                double qpos_value) const;
    // (clipped torques in joints() order, names that were limited).
    [[nodiscard]] std::pair<std::vector<double>, std::vector<std::string>>
    strength_limited(std::span<const double> torques,
                     std::span<const double> qpos,
                     std::span<const double> qvel) const;
    [[nodiscard]] std::vector<std::string>
    strength_violations(const NamedEntries<double> &active_torques,
                        std::span<const double> qpos,
                        std::span<const double> qvel) const;

    // rider_effort.py solved_effort — updates last_terms + effort
    // diagnostics from solved actuator forces at the incoming interval.
    [[nodiscard]] EffortDiagnostics
    solved_effort(const mjData *data, std::span<const double> incoming_qpos,
                  std::span<const double> incoming_velocity, double dt_s);

    // Owned-state surface — the fields the bootstrap wire carries.
    struct State {
        bool enabled = true;
        bool command_enabled = true;
        std::vector<double> active_state;
        std::optional<double> activation_time_s;
        NamedEntries<JointTerms> last_terms;
        NamedEntries<bool> saturated_ik;
        NamedEntries<bool> ik_reach_limited;
        NamedEntries<double> joint_torques_nm;
        NamedEntries<double> joint_capacity_nm;
        std::optional<double> lean_limit_rad;
        NamedEntries<PedalRecoveryState> pedal_recovery;
        EffortDiagnostics effort_diagnostics;
        AllocationDiagnostics allocation_diagnostics;
        SupportDiagnostics support_diagnostics;
        NamedEntries<SoleGoalDiagnostics> sole_goal_diagnostics;
    };
    [[nodiscard]] State state() const;
    void restore(const State &state);

    [[nodiscard]] const std::vector<JointEntry> &joints() const {
        return joints_;
    }
    [[nodiscard]] bool spindle() const { return true; }
    [[nodiscard]] bool welded() const { return false; }
    [[nodiscard]] bool command_enabled() const { return command_enabled_; }
    [[nodiscard]] std::optional<double> lean_limit_rad() const {
        return lean_limit_rad_;
    }

private:
    // _targets() spindle branch — IK goals for one leg, updates the
    // sole/IK diagnostic mirrors.
    [[nodiscard]] spindle::Vec2 leg_targets(const mjData *data,
                                            const char *side);
    // _upper_targets() welded-grip + spindle branch.
    [[nodiscard]] std::vector<std::pair<std::string, double>>
    upper_targets(const mjData *data, const RiderPosture *posture);
    // _predict_target_state — detached kinematics on a scratch mjData.
    void predict_target_state(const mjData *data, mjData *scratch,
                              bool reverse) const;
    // _directional_limit: strength capacity or the flat joint ceiling.
    [[nodiscard]] double directional_limit(const std::string &name,
                                           const mjData *data,
                                           double sign) const;
    // limit_torques — per-joint directional+speed cap then power budget.
    [[nodiscard]] std::vector<double>
    limit_torques(std::span<const double> torques, const mjData *data) const;
    // finalize_effort — activation/bounds re-check plus diagnostics.
    [[nodiscard]] std::vector<double>
    finalize_effort(mjData *data, std::span<const double> torques,
                    bool advance, double dt_s, bool steady_state);

    const mjModel *model_;
    SpindlePose pose_;
    SpindleConfig config_;
    double crank_length_m_;
    std::vector<JointEntry> joints_;
    std::vector<std::string> locked_joints_;
    // name -> (lo,hi) for jnt_limited joints; aligned vectors for
    // envelope_forces (joint_ranges order = joints_ order filtered).
    std::map<std::string, spindle::Vec2> joint_ranges_;
    std::vector<int> envelope_qpos_adrs_;
    std::vector<int> envelope_dof_adrs_;
    std::vector<double> envelope_lower_;
    std::vector<double> envelope_upper_;

    int pelvis_body_ = -1;
    int frame_body_ = -1;
    int torso_body_ = -1;
    int steer_body_ = -1;
    int crank_body_ = -1;
    int crank_joint_ = -1;
    int torso_joint_ = -1;
    int frame_pitch_dof_ = -1;
    int rider_pitch_dof_ = -1;
    int crank_spin_dof_ = -1;
    int crank_spin_qpos_ = -1;
    std::array<int, 2> pedal_spin_dofs_{-1, -1};
    std::array<int, 2> feet_bodies_{-1, -1};
    std::array<int, 2> sole_sites_{-1, -1};
    std::array<int, 2> pedal_sites_{-1, -1};
    std::array<int, 2> pedal_geoms_{-1, -1};
    std::array<int, 2> pedal_bodies_{-1, -1};
    std::array<int, 2> hip_joints_{-1, -1};
    std::array<int, 2> upper_arm_bodies_{-1, -1};
    std::array<int, 2> forearm_bodies_{-1, -1};
    std::array<int, 2> grip_sites_{-1, -1};
    std::array<std::array<double, 3>, 2> weld_grip_offset_{};
    std::array<spindle::LegLoopGeometry, 2> leg_geometry_{};
    double arm_upper_m_ = 0.;
    double arm_lower_m_ = 0.;
    double arm_upper_angle0_ = 0.;
    double arm_lower_angle0_ = 0.;
    int arm_branch_ = 1;
    double neutral_torso_q_ = 0.;
    bool neutral_torso_saturated_ = false;
    std::optional<double> lean_limit_rad_;
    std::vector<int> rider_bodies_;
    double rider_mass_ = 0.;
    // legs by side: joint indices into joints_ for ('hip','knee'[, 'ankle'])
    std::array<std::vector<std::size_t>, 2> leg_joint_index_{};

    // Mutable controller state (the bootstrap-owned fields). Ordered
    // entries preserve the oracle's dict insertion order on the wire.
    bool enabled_ = true;
    bool command_enabled_ = true;
    std::vector<double> active_state_;
    std::optional<double> activation_time_s_;
    NamedEntries<JointTerms> last_terms_;
    NamedEntries<bool> saturated_ik_{{"front", false}, {"rear", false}};
    NamedEntries<bool> ik_reach_limited_{{"front", false}, {"rear", false}};
    NamedEntries<double> joint_torques_nm_;
    NamedEntries<double> joint_capacity_nm_;
    NamedEntries<PedalRecoveryState> pedal_recovery_;
    EffortDiagnostics effort_diagnostics_;
    AllocationDiagnostics allocation_diagnostics_;
    SupportDiagnostics support_diagnostics_;
    NamedEntries<SoleGoalDiagnostics> sole_goal_diagnostics_;
    NamedEntries<std::array<double, 3>> sole_targets_;
    double target_difference_s_ = 1e-4;

    std::unique_ptr<mjData, decltype(&mj_deleteData)> target_data_;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> previous_target_data_;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> coasting_target_data_;
};

} // namespace rider
