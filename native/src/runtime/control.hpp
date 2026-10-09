// runtime/control.hpp — the owned step's helper layer (plan A3).
//
// Pure mjModel/mjData + typed state ports of the small oracle modules the
// physical step consumes: control_clock.py, contact_filter.py,
// contacts.py's state faces, physical_crash.py, balance_monitor.py,
// virtual_rider.py's CrashDetector, control.py's RideControl contract,
// rider_state.py's road sampling and force_accumulator.py. No nanobind
// types enter this header — wire decode lives in control.cpp and the
// binding TUs; every method mirrors the Python statements it names and
// the Python ordering fixes the FP contract.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "../rider/intent.hpp"
#include "../rider/spindle_controller.hpp"
#include "../writers/tire.hpp"

namespace runtime {

// ---- Wire: the insertion-ordered plain-data tree the step metadata
// carries (channels/diagnostics/drive maps). WireObject preserves Python
// dict insertion order; a std::map would silently reorder. monostate is
// the None lane; booleans stay distinct from integers like Python's
// bool/int split.
struct Wire;
using WireArray = std::vector<Wire>;
using WireObject = std::vector<std::pair<std::string, Wire>>;

struct Wire {
    std::variant<std::monostate, bool, std::int64_t, double, std::string,
                 WireArray, WireObject>
        value;
    Wire() : value(std::monostate{}) {}
    // Implicit conversions are the point: the wire tree mirrors Python's
    // dynamically typed plain values, so every alternative constructs
    // transparently like a literal.
    // NOLINTBEGIN(cppcoreguidelines-explicit-constructor,misc-explicit-constructor)
    Wire(std::nullptr_t) : value(std::monostate{}) {}
    Wire(bool b) : value(b) {}
    Wire(int i) : value(static_cast<std::int64_t>(i)) {}
    Wire(unsigned i) : value(static_cast<std::int64_t>(i)) {}
    Wire(std::int64_t i) : value(i) {}
    Wire(double x) : value(x) {}
    Wire(const char *s) : value(std::string(s)) {}
    Wire(std::string s) : value(std::move(s)) {}
    Wire(std::string_view s) : value(std::string(s)) {}
    Wire(WireArray a) : value(std::move(a)) {}
    Wire(WireObject o) : value(std::move(o)) {}
    // NOLINTEND(cppcoreguidelines-explicit-constructor,misc-explicit-constructor)
};

// np.asarray(vec).tolist() — every element a Python float.
[[nodiscard]] Wire wire_array(std::span<const double> values);
// drivetrain::DiagnosticValue -> Wire (monostate stays None).
[[nodiscard]] Wire
wire_diagnostics(const std::map<std::string,
                 std::variant<std::monostate, double, int, bool,
                              std::string>> &values);

// ---- control.py RideControl ------------------------------------------
// The full wire shape is rider::IntentControl; decode (and its
// __post_init__ gates) live in rider/intent_wire.cpp. validate_for
// (control.py:35-43) additionally needs the physics/drive mode and the
// rider variant — the runtime config already pins physics_mode='physical'
// and drive_mode='articulated_effort', so the effort-mode gate is
// discharged at construction and only the articulated_planar gate is
// live here.
using RideControl = rider::IntentControl;

// control.py:35-43 validate_for — caller supplies whether the rider is
// articulated_planar (runtime: the rider-control writer ensemble exists).
void control_validate_for(const RideControl &control,
                          bool articulated_planar);

// ---- control_clock.py ------------------------------------------------
class ControlClock {
public:
    // __init__ — positive finite durations, integer-step period.
    ControlClock(double timestep, double period);

    // is_tick — a Python int step, so signedness is checked explicitly.
    [[nodiscard]] bool is_tick(std::int64_t step) const;

    // hold()/held() — the held command dict; held() raises
    // std::runtime_error before the first tick like the oracle's
    // RuntimeError.
    void hold(rider::NamedEntries<double> torques);
    [[nodiscard]] const rider::NamedEntries<double> &held() const;
    void reset() noexcept { held_.reset(); }
    [[nodiscard]] const std::optional<rider::NamedEntries<double>> &
    held_state() const noexcept {
        return held_;
    }

    // Python-attribute mirror (control_clock fields); public by contract.
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
    double timestep_s;
    double period_s;
    std::int64_t steps_per_period;
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

private:
    std::optional<rider::NamedEntries<double>> held_;
};

// ---- contact_filter.py ------------------------------------------------
struct GroundedFilter {
    // __init__(hold_s) validates finite nonnegative.
    explicit GroundedFilter(double hold);
    void reset() noexcept;
    bool update(bool raw_grounded, double time_s);

    // state_dict() — {'last_time_s','last_loaded_s','value'}; hold_s is
    // config-owned (monitors.grounded_hold_s) and never serialized.
    struct State {
        std::optional<double> last_time_s, last_loaded_s;
        bool value = false;
    };
    [[nodiscard]] State state() const noexcept {
        return {.last_time_s = last_time_s,
                .last_loaded_s = last_loaded_s,
                .value = value};
    }
    void restore(const State &s) noexcept {
        last_time_s = s.last_time_s;
        last_loaded_s = s.last_loaded_s;
        value = s.value;
    }

    double hold_s;
    std::optional<double> last_time_s;
    std::optional<double> last_loaded_s;
    bool value = false;
};

// ---- contacts.py ------------------------------------------------------
constexpr int kContactDropoutSteps = 10;
constexpr double kContactLoadThresholdN = 1.0;

// _BridgedLoad — one wheel's contact load held across a dropout run.
struct BridgedLoad {
    explicit BridgedLoad(int steps);
    double update(double raw_load_n) noexcept;
    void reset() noexcept;

    // state_dict leaf — {'held_load_n','unloaded_steps'}; dropout_steps is
    // construction config, never serialized.
    struct State {
        double held_load_n = 0.;
        std::int64_t unloaded_steps = 0;
    };
    [[nodiscard]] State state() const noexcept {
        return {.held_load_n = held_load_n,
                .unloaded_steps = unloaded_steps};
    }
    void restore(const State &s) noexcept {
        held_load_n = s.held_load_n;
        unloaded_steps = s.unloaded_steps;
    }

    int dropout_steps;
    double held_load_n = 0.;
    std::int64_t unloaded_steps = 0;
};

// contacts.py TerrainContacts — the durable per-step contact view. The
// snapshot slots collapse to their only consumed field: the wire state
// (setup.py _contacts_state) carries front/rear_slip_mps only, and the
// physical step's sole post-restore read is rear_snapshot.slip_mps
// (physical_runtime.py:259). The full TireSnapshot pair travels beside
// it as the step's `snapshots` local.
struct RuntimeContacts {
    double front_load_n = 0., rear_load_n = 0.;
    double front_support_n = 0., rear_support_n = 0.;
    double handlebar_load_n = 0.;
    std::optional<double> front_slip_mps, rear_slip_mps;
    std::optional<bool> front_controller_grounded, rear_controller_grounded;

    [[nodiscard]] bool front_in_contact() const noexcept {
        return front_load_n > kContactLoadThresholdN;
    }
    [[nodiscard]] bool rear_in_contact() const noexcept {
        return rear_load_n > kContactLoadThresholdN;
    }
    [[nodiscard]] bool airborne() const noexcept {
        return !front_in_contact() && !rear_in_contact();
    }
    [[nodiscard]] bool handlebar_in_contact() const noexcept {
        return handlebar_load_n > kContactLoadThresholdN;
    }
};

// contacts.py TerrainContactQuery, minus the legacy wheel-load path: the
// supported compliant_2d backend always reaches the tire branch of
// _contacts(), so query()/_sum_normal_loads' per-patch assembly is never
// invoked natively — the held-load bridges and controller debounce still
// exist because the bootstrap round-trips their state_dict().
class RuntimeContactQuery {
public:
    // __init__ resolves the terrain/contact geoms once; dropout_steps
    // validation matches the oracle.
    explicit RuntimeContactQuery(const mjModel *model,
                                 int dropout_steps = kContactDropoutSteps);

    // handlebar_load (contacts.py:279-300) — stateless normal-magnitude
    // sum over handlebar-terrain rows; no bridge/filter advances.
    [[nodiscard]] double handlebar_load(const mjModel *model,
                                        const mjData *data) const;

    void reset() noexcept;

    // Python-attribute mirror — the bridges/filters the step pipeline
    // reaches directly (contacts.py field names).
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
    // state_dict() round-trip — contacts.py:267-277.
    BridgedLoad front_load{0}, rear_load{0};
    GroundedFilter front_controller{0.005}, rear_controller{0.005};
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

    struct State {
        BridgedLoad::State front_load, rear_load;
        GroundedFilter::State front_controller, rear_controller;
    };
    [[nodiscard]] State state() const noexcept {
        return {.front_load = front_load.state(),
                .rear_load = rear_load.state(),
                .front_controller = front_controller.state(),
                .rear_controller = rear_controller.state()};
    }
    void restore(const State &s) noexcept {
        front_load.restore(s.front_load);
        rear_load.restore(s.rear_load);
        front_controller.restore(s.front_controller);
        rear_controller.restore(s.rear_controller);
    }

    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
    int front_id = -1, rear_id = -1, handlebar_id = -1;
    std::vector<int> terrain_ids;
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

private:
    mutable std::array<double, 6> force_{}; // mj_contactForce scratch
};

// ---- physical_crash.py ------------------------------------------------
// _crash_geom_ids is constructor-resolved rather than function-cached —
// one physical step owns exactly one model, so no id(model) trick is
// needed. Returns the cause string or nullopt.
[[nodiscard]] std::optional<std::string>
physical_contact_crash(const mjModel *model, const mjData *data,
                       const std::vector<int> &catch_ids,
                       const std::vector<int> &terrain_ids,
                       const std::vector<int> &rider_ids,
                       double load_threshold_n = 1.0);

// The crash-relevant geom id sets (catch_plane / terrain / geom_rider_*),
// resolved once at construction.
struct CrashGeomIds {
    std::vector<int> catch_ids, terrain_ids, rider_ids;
};
[[nodiscard]] CrashGeomIds crash_geom_ids(const mjModel *model);

// ---- balance_monitor.py ----------------------------------------------
struct BalanceLostEvent {
    double time_s = 0., position_m = 0., speed_mps = 0.;
};

class BalanceMonitor {
public:
    BalanceMonitor(double floor, double dwell, double grace);
    void reset() noexcept;
    // update() returns the latched/created event (nullopt while upright).
    std::optional<BalanceLostEvent> update(double time_s, double position_m,
                                         double speed_mps, bool riding);
    // observation() — the rider_balance channel dict.
    [[nodiscard]] WireObject observation() const;

    // state_dict() — balance_monitor.py:62-72; the config triple stays
    // construction-owned.
    struct State {
        std::optional<BalanceLostEvent> event;
        double low_speed_s = 0., grace_left_s = 0.;
        std::optional<double> started_s, low_started_s, last_time_s;
    };
    [[nodiscard]] State state() const noexcept {
        return {.event = event,
                .low_speed_s = low_speed_s,
                .grace_left_s = grace_left_s,
                .started_s = started_s,
                .low_started_s = low_started_s,
                .last_time_s = last_time_s};
    }
    void restore(const State &s) noexcept {
        event = s.event;
        low_speed_s = s.low_speed_s;
        grace_left_s = s.grace_left_s;
        started_s = s.started_s;
        low_started_s = s.low_started_s;
        last_time_s = s.last_time_s;
    }

    double floor_mps, dwell_s, grace_s;
    std::optional<BalanceLostEvent> event;
    double low_speed_s = 0.;
    double grace_left_s = 0.;
    std::optional<double> started_s, low_started_s, last_time_s;
};

// ---- virtual_rider.py CrashDetector -----------------------------------
struct CrashEvent {
    std::string cause;
    double time_s = 0., position_m = 0., pitch_rad = 0.;
};

class CrashDetector {
public:
    // __init__ resolves root_x/root_pitch qpos addresses; the 60 deg
    // default matches CRASH_PITCH_LIMIT_DEG.
    explicit CrashDetector(const mjModel *model,
                           double pitch_limit_deg = 60.0);
    // check() latches the first cause; a latched event sticks.
    std::optional<CrashEvent> check(const mjData *data,
                                    const RuntimeContacts &contacts);
    void reset() noexcept { event_.reset(); }
    [[nodiscard]] const std::optional<CrashEvent> &event() const noexcept {
        return event_;
    }
    void set_event(std::optional<CrashEvent> event) {
        event_ = std::move(event);
    }

    // Constructor-resolved model lookups — oracle attribute names.
    // NOLINTBEGIN(cppcoreguidelines-non-private-member-variables-in-classes)
    int pitch_qposadr = 0, root_x_qposadr = 0;
    double pitch_limit_rad = 0.;
    // NOLINTEND(cppcoreguidelines-non-private-member-variables-in-classes)

private:
    mjtSize nq_ = 0;
    std::optional<CrashEvent> event_;
};

// ---- rider_state.py ----------------------------------------------------
struct RoadSample {
    double x_m = 0., height_m = 0., grade = 0.;
};

// road_samples (rider_state.py:56-88): wheel positions + lookahead window
// at .05 spacing, np.interp heights and per-segment grades.
[[nodiscard]] std::vector<RoadSample>
road_samples(std::span<const std::array<double, 2>> vertices,
             std::span<const double> wheel_x_m, double lookahead_m,
             double spacing_m = .05);

[[nodiscard]] double road_grade_for_posture(std::span<const RoadSample> road);
[[nodiscard]] double road_grade_preview(std::span<const RoadSample> road);

// RiderKinematicState — qpos/qvel snapshots plus the road window (the
// dataclass freezes its inputs; the native form owns them).
struct RiderKinematicState {
    std::vector<double> qpos, qvel;
    double rider_time_s = 0.;
    std::vector<RoadSample> road;
};

[[nodiscard]] RiderKinematicState rider_kinematic_state(
    const mjModel *model, const mjData *data,
    std::span<const std::array<double, 2>> vertices,
    std::span<const double> wheel_x_m, double lookahead_m,
    double spacing_m = .05);

// ---- force_accumulator.py ---------------------------------------------
// Insertion-ordered (name -> nv vector) with the oracle's add() gates:
// exact width, all-finite, no duplicate names.
class ForceAccumulator {
public:
    explicit ForceAccumulator(mjtSize nv) : nv_(nv) {}
    void clear() noexcept { components_.clear(); }
    void add(std::string_view name, std::span<const double> qfrc);
    void add(std::string_view name, const std::vector<double> &qfrc) {
        add(name, std::span<const double>(qfrc));
    }
    [[nodiscard]] const std::vector<std::pair<std::string,
                                              std::vector<double>>> &
    components() const noexcept {
        return components_;
    }
    [[nodiscard]] const std::vector<double> *
    component(std::string_view name) const {
        for (const auto &[n, v] : components_)
            if (n == name) return &v;
        return nullptr;
    }
    // total() — zero-init then sequential in-place adds in insertion
    // order (the P2 ordering contract Stepper::total documents).
    [[nodiscard]] std::vector<double> total() const;

private:
    mjtSize nv_;
    std::vector<std::pair<std::string, std::vector<double>>> components_;
};

// ---- _runtime_state 'energy' ------------------------------------------
// The bootstrap's energy section — baseline datums plus the running
// counters A4's period close owns. A3 restores and round-trips every
// field; only the baselines are read inside the step boundary (the
// cumulative counters move in _close_period — not yet ported).
struct EnergyState {
    std::optional<double> initial_energy_j;
    double energy_scale_j = 0.;
    double loss_j = 0., active_work_j = 0., external_work_j = 0.;
    double solver_work_j = 0.;
    double muscle_signed_j = 0., muscle_positive_j = 0.;
    double motor_signed_j = 0., motor_positive_j = 0.;
    double constraint_absolute_j = 0.;
    std::optional<double> initial_battery_j;
    double electrical_work_j = 0.;
    double mechanical_energy_j = 0.;
    WireObject elastic_energy_j;
    double residual_j = 0.;
};

} // namespace runtime
