#pragma once
#include "../drivetrain/freehub.hpp"
#include "../drivetrain/motor.hpp"
#include "../drivetrain/pedaling.hpp"
#include "../drivetrain/shifting.hpp"
#include "../drivetrain/transmission.hpp"
namespace drivetrain {
class ArithmeticError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};
struct DriveConfig {
    DrivePolicyConfig policies;
    std::string drive_mode, transmission_model;
    double human_torque_nm{}, torque_ripple{}, crank_phase_rad{}, chain_k_n_m{},
        chain_c_ns_m{}, bearing_c_nms_rad{}, rotor_inertia_kgm2{};
    bool motor_clutch{};
};
struct RideControl {
    std::optional<double> motor_torque_nm, motor_limit_nm, human_torque_nm;
    bool rider_enabled = true;
};
struct PendingActuation {
    double requested{}, omega{}, dt{};
    bool enabled{};
};
struct DriveSnapshot {
    PedalingSnapshot pedaling;
    ShiftingSnapshot shifting;
    AssistSnapshot assist;
    BatterySnapshot battery;
    std::optional<FreehubSnapshot> hub;
    std::optional<double> shift_time_s, last_time_s, reference, psi;
    std::optional<std::vector<double>> angles;
    Diagnostics last;
    std::optional<Diagnostics> probe_last;
    std::optional<PendingActuation> pending_actuation;
    std::optional<TransmissionSnapshot> ideal_hub, clutch, freewheel;
};
using ForceComponents = std::vector<std::pair<std::string, std::vector<double>>>;
class DrivetrainWriter {
  public:
    DrivetrainWriter(mjModel *model, mjData *data, DriveConfig config);
    void reset();
    void restart_clock();
    PedalingState prepare(const RideControl &control, double dt, bool braking,
                          bool active, bool advance, bool contact,
                          std::optional<double> slip, std::optional<double> ceiling);
    const ForceComponents &components(const RideControl &control, double dt,
                                      double speed, bool braking, bool active,
                                      bool advance, double sensed,
                                      std::optional<PedalingState> pedaling,
                                      bool contact, std::optional<double> slip);
    std::span<const double> settle();
    const Diagnostics &diagnostics(bool probe) const;
    Diagnostics stored_energy();
    DriveSnapshot state() const;
    void restore(const DriveSnapshot &snapshot);
    const Transmission *transmission_storage() const { return ideal_hub_.get(); }

  private:
    mjModel *model_;
    mjData *data_;
    DriveConfig config_;
    bool simplified_, effort_;
    GeometryWorkspace geometry_;
    int frame_, crank_, rear_, cassette_{};
    struct Joint {
        int qpos{}, dof{};
    };
    std::map<std::string, Joint> joints_;
    std::vector<Joint> bearing_joints_;
    int human_actuator_, motor_actuator_;
    PedalingPolicy pedaling_;
    CadenceShifter shifting_;
    AssistController assist_;
    Battery battery_;
    std::optional<Freehub> hub_;
    std::unique_ptr<Transmission> ideal_hub_, clutch_, freewheel_;
    DriveSnapshot live_;
    ForceComponents components_;
    std::vector<double> transmission_;
    double angle(int body, std::optional<double> reference = std::nullopt);
    void shifts(Diagnostics &d) const;
    const ForceComponents &compute(const RideControl &, double, double, bool, bool,
                                   bool, double, std::optional<PedalingState>, bool,
                                   std::optional<double>);
};
} // namespace drivetrain
using drivetrain::DrivetrainWriter;
