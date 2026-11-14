#pragma once
#include "common.hpp"
#include "../initialization/setup_owner.hpp"
#include "../rider/intent.hpp"
#include "../runtime/control.hpp"
#include "../runtime/research_metrics.hpp"
#include "../runtime/samples.hpp"
#include "../runtime/static_brake.hpp"
#include "../stepper.hpp"
#include <functional>

namespace runtime { class NativeResearchRuntime; }
namespace bike::validation {
// Used only by cold validation rigs. It does not expose a writable engine
// handle through the application API. Initial-condition changes require t=0;
// advance validates the ordinary physical input boundary and retains the
// normal accounting/poison/publication flow.
class RuntimeRigAccess {
public:
    static void inspect(runtime::NativeRideRuntime&,const std::function<void(const Stepper&,const runtime::PhysicalStep&)>&);
    static Value rebase_airborne(runtime::NativeRideRuntime&,double height,double vertical_speed,
        bool release_rider,bool zero_velocity,bool spin_wheels,bool reset_tire);
    static Value static_metrics(runtime::NativeRideRuntime&,const Value& setup_metadata,double expected_mass);
    static runtime::TruthGeometry truth_geometry(runtime::NativeRideRuntime&);
    static initialization::PhysicalInitialState initial_state(runtime::NativeRideRuntime&,
        const initialization::SetupRequest&,const initialization::EquilibriumResult&,const Value& metadata,
        std::optional<int> station_count={});
    static void external_interval(runtime::NativeRideRuntime&,std::span<const double> force);
    // physical.rider_intent.schedule_pulse — the oracle's pre-episode
    // body-thrust schedule call, routed through the same guarded access.
    static void schedule_rider_pulse(runtime::NativeRideRuntime&,double start_s,double duration_s,double amplitude_rad);
    // rider_intent.py signals_from_channels — the production
    // PhysicalStep::update_intent_signals fold evaluated on a
    // caller-supplied sensor/tire channel dict; returns the latched
    // intent signals like the oracle's return value.
    static ::rider::IntentSignals fold_intent_signals(runtime::NativeRideRuntime&,const runtime::WireObject& sensors_and_tires);
    // Launch part 1, task 4 — installs the test-only rider command set on
    // the physical step; std::nullopt releases it so the intent's own
    // (empty) commands resolve again. Applies to the next physical step —
    // rider commands are born inside the step, never queued through the
    // caller's control path.
    static void set_rider_commands(runtime::NativeRideRuntime&,std::optional<::rider::RiderCommands>);
    // Same override on a research runtime's owned physical runtime — the
    // commands bypass the motor-command delay queue by design (they are
    // step-internal), so the journal and the channel observe them at the
    // next committed interval.
    static void set_rider_commands(runtime::NativeResearchRuntime&,std::optional<::rider::RiderCommands>);
    // Launch part 1, task 6 — read-only handle on the physical step's
    // static brake so the latch tests snapshot its committed State (and
    // copy it for continuation) without a writable component handle.
    static const runtime::StaticBrake &static_brake(const runtime::PhysicalStep&);
    // Energy-gate regression seam — feeds one caller-built sample through
    // the episode's ordinary consume path so tests exercise the measured/
    // unmeasured energy-quality boundary without a physical engine run.
    static void consume_sample(runtime::NativeResearchRuntime&,const runtime::SamplePtr&,bool stop_at_outcome);
private:
    // -O0 frame budget: the rebase_airborne mutation stays under
    // -Wframe-larger-than by routing the multi-kilobyte rider-contact
    // state/probe and energy-state temporaries through these helpers.
    static void release_rider(Stepper&,runtime::PhysicalStep&);
    static void reset_rider_contacts(Stepper&);
    static Value rebase_energy(runtime::PhysicalStep&);
};
} // namespace bike::validation
