#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>
#include "../initialization/legacy_equilibrium.hpp"
#include "../io/value.hpp"
#include "../model/builder.hpp"
#include "../model/suspension_setup.hpp"
#include "../runtime/runtime.hpp"

namespace bike::legacy {
struct SuspensionForce { double total_n, spring_n, damper_n; };
class SuspensionController {
public:
    SuspensionController(const geometry::BikeSpecs&,const model::ForkAirSpring&,
        const model::BikeSuspensionSystem&);
    [[nodiscard]] SuspensionForce compute_fork_force(double travel_mm,double velocity_mps) const;
    [[nodiscard]] SuspensionForce compute_shock_force(double stroke_mm,double velocity_mps) const;
private:
    const geometry::BikeSpecs& specs_;
    const model::ForkAirSpring& air_;
    const model::BikeSuspensionSystem& suspension_;
};
struct SimModelHandles {
    int main_pivot=-1,shock=-1,fork=-1,p3_ss=-1,p3_rocker=-1,bb=-1,pfa=-1,pra=-1;
    [[nodiscard]] static SimModelHandles from_model(const mjModel&);
};
class TelemetryProvider {
public:
    TelemetryProvider(const geometry::BikeSpecs&,const kinematics::HorstLinkageSolver&,
        const model::ForkAirSpring&,const model::BikeSuspensionSystem&);
    // The caller supplies the already-computed state. Observing telemetry
    // never forwards, steps or mutates the authoritative mjData.
    [[nodiscard]] io::Value compute(const mjModel&,const mjData&,double target_rear_mm,
        bool include_rider,const SimModelHandles&,std::string_view camera_mode="2d") const;
private:
    const geometry::BikeSpecs& specs_;
    const kinematics::HorstLinkageSolver& solver_;
    const model::ForkAirSpring& air_;
    const model::BikeSuspensionSystem& suspension_;
};
struct PlaygroundOptions {
    geometry::BikeSpecs specs;
    bool include_rider=false;
    std::string model_mode="playground";
    std::filesystem::path asset_root;
};
struct PlaygroundSnapshot {
    double time_s=0.,target_rear_travel_mm=0.,target_fork_travel_mm=0.;
    double timestep_s=0.;
    bool include_rider=false,auto_sweep=false,debug_markers=false,show_telemetry=true;
    std::uint64_t generation=0;
    std::string model_mode,camera_mode;
    std::vector<double> qpos,qvel,ctrl;
    std::vector<double> integration_state;
    std::array<std::vector<double>, 6> model_fields;
    io::Value telemetry;
};
// The controller owns its model/data and serializes queued inputs with each
// step. It depends on no window system and creates no thread.
class Playground {
public:
    ~Playground();
    Playground(const Playground&)=delete;
    Playground& operator=(const Playground&)=delete;
    void step();
    void step_at(double wall_time_s);
    void reset();
    void set_model(std::string mode);
    void set_travel(double rear_mm,double fork_mm);
    void set_auto_sweep(bool enabled,double wall_time_s,double frequency_hz=.4);
    [[nodiscard]] bool toggle_rider();
    [[nodiscard]] bool toggle_debug_markers();
    void set_fork_pressure(double psi);
    void set_fork_tokens(int tokens);
    void set_fork_clicks(std::optional<int> hsc,std::optional<int> lsc,std::optional<int> rebound);
    void set_shock_clicks(std::optional<int> hsc,std::optional<int> lsc,std::optional<int> rebound,
        std::optional<int> hbo={},std::optional<bool> lockout={});
    void apply_preset(model::DamperPreset);
    [[nodiscard]] model::DamperPreset cycle_preset(bool forward=true);
    void set_camera_mode(std::string_view);
    void enqueue_key(int keycode);
    // Returns false for an unhandled key. Window and help rendering are T48.
    [[nodiscard]] bool handle_key(int keycode,double wall_time_s);
    [[nodiscard]] PlaygroundSnapshot snapshot() const;
    [[nodiscard]] io::Value telemetry() const;
    [[nodiscard]] runtime::ModelImage model_image() const;
private:
    struct Impl;
    explicit Playground(std::unique_ptr<Impl>);
    std::unique_ptr<Impl> impl_;
    friend std::unique_ptr<Playground> make_playground(PlaygroundOptions);
};
[[nodiscard]] std::unique_ptr<Playground> make_playground(PlaygroundOptions options={});
void step(Playground&);
void reset(Playground&);
// Re-export the same ride equilibrium owner used by RideSimulation; no
// second relaxation algorithm or second physical stepping path exists.
[[nodiscard]] initialization::StaticEquilibriumResult solve_static_equilibrium(
    initialization::StaticEquilibriumContext&,const initialization::StaticEquilibriumOptions& = {});
void apply_stand_targets(const mjModel&,mjData&,const kinematics::HorstLinkageSolver&,
    int rear_actuator,int fork_actuator,double rear_mm,double fork_mm);
void reset_stand(const mjModel&,mjData&,const kinematics::HorstLinkageSolver&,int rear_actuator,int fork_actuator);
} // namespace bike::legacy
