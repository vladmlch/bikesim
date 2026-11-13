// Completion 3.34 (T53). Source: tests/reference/test_pneumatic_physical_runtime.py.
//
// Only the end-to-end record ports here. The oracle rides the pneumatic
// backend 400 steps on a flat/coast/lumped plant built from
// SimulationPhysicsConfig('physical', drive_mode='coast', tires=...); the
// native twin writes exactly those non-default fields into a physics TOML
// and runs it through the production resolve_ride -> app::prepare ->
// make_ride -> advance pipeline on --track flat --rider lumped, with
// --check-policy diagnostic standing in for set_strict(False) — the same
// documented constraint-work budget rationale both sides disable. The
// shipped research profile is not a stand-in: its articulated rider and
// [seated_climb] block are a different plant, and its historical pose npz
// requires legacy source provenance the oracle's internal settle never
// consults.
// The two codec records are already covered `contract`:
//   * ...round_trips_through_the_cache_codec ->
//     reference.pneumatic_tire_forces::test_state_wire_names_and_round_trip;
//   * ...round_trips_through_the_npz_cache_file ->
//     reference.equilibrium_state_file's state_tire_wire npz coverage.
#include "support/fixture_test.hpp"
#include "recording_test_support.hpp"
#include "app/prepare.hpp"
#include "cli/application.hpp"
#include "cli/physical_capture.hpp"
#include "io/value.hpp"
#include "runtime/runtime.hpp"
#include "runtime/samples.hpp"
#include <string>
#include <vector>

namespace {

namespace io = bike::io;
namespace sw = runtime::sample_wire;

const io::Value::Object &object_of(const io::Value &v, std::string_view what) {
    const auto *o = std::get_if<io::Value::Object>(&v.data);
    bt::require(o != nullptr, std::string(what) + " is an object");
    return *o;
}
const io::Value &member(const io::Value &v, std::string_view key) {
    const auto *found = io::find_member(object_of(v, "member"), key);
    bt::require(found != nullptr, "member present: " + std::string(key));
    return *found;
}
double number(const io::Value &v) { return io::as_double(v, "number"); }
const runtime::WireObject &wire_obj(const runtime::WireObject &o,
                                    std::string_view key) {
    return sw::object(sw::required(o, key));
}

// test_pneumatic_backend_runs_a_physical_ride_end_to_end — 400 committed
// steps on the pneumatic profile, then the equilibrium envelope plus the
// retained tires/energy channels the oracle asserted.
void test_pneumatic_backend_runs_a_physical_ride_end_to_end() {
    // SimulationPhysicsConfig('physical', drive_mode='coast', tires=
    // TireBackendConfig(backend='pneumatic', surface_mode='track',
    // pneumatic=PneumaticTireConfig(tier='fast'))) — every other field
    // stays at its shared default, including equilibrium_cache_enabled.
    const bt::TemporaryDirectory dir("pneumatic_ride");
    const auto physics = dir.path() / "physics.toml";
    {
        std::ofstream out(physics);
        out << "physics_mode = \"physical\"\n"
               "drive_mode = \"coast\"\n"
               "timestep_s = 0.0005\n"
               "control_period_s = 0.005\n"
               "[rider_launch]\nstart_cog_grade_breaks = []\nstart_cog_teeth = [24]\nstart_pedal_angle_rad = 0.0\nstart_lean_grades = [0.0]\nstart_lean_rad = [0.0]\nplan_enabled = false\nstandstill_speed_mps = 0.1\nstandstill_crank_rpm = 5.0\neffort_rate_nm_s = 40.0\neffort_cap_asphalt_nm = 60.0\neffort_cap_hardpack_nm = 60.0\neffort_cap_loose_nm = 60.0\neffort_cap_wet_nm = 60.0\neffort_cap_scree_nm = 60.0\neffort_cap_grades = [0.0]\neffort_cap_grade_nm = [1000.0]\neffort_rise_speed_mps = 2.5\nrelease_torque_grades = [0.0, 0.2]\nrelease_torque_nm = [0.0, 30.0]\nbrake_release_s = 0.15\nmax_hold_s = 1.5\nupshift_crank_rpm = 82.0\nshift_retry_s = 0.25\nend_speed_mps = 3.0\nend_crank_rpm = 60.0\nend_hold_s = 0.3\nmax_launch_s = 6.0\nhandover_s = 0.9\n[front_lift_reaction]\nhand_window_s = 0.5\nhand_lag_s = 0.5\nhand_drop_ratio = 0.7\npitch_rate_up_rad_s = 0.5\npitch_rad = 0.17\ndecision_s = 0.1\nease_enabled = false\nlean_enabled = false\nrear_brake_tap_enabled = false\nease_fraction = 0.7\nease_rate_per_s = 5.0\nlean_add_rad = 0.1\nlean_rate_rad_s = 0.5\nclear_s = 0.3\n[monitors]\n"
               "grounded_hold_s = 0.005\n"
               "contact_dropout_s = 0.005\n"
               "[seated_climb]\n"
               "period_s = 0.00125\nreaction_delay_s = 0.15\nlean_rate_rad_s = 0.5\norientation_tau_s = 0.5\nsurge_budget_s = 15\ntrim_dead_time_s = 0.2\nmin_elbow_flexion_deg = 15.0\nlaunch_effort_ramp_s = 1.5\nlaunch_min_grade = 0.03\nlaunch_standstill_crank_rate_rad_s = 1.0\n"
               "[drive]\nmotor_clutch = false\n"
               "[drive.assist]\nrelease_hold_s = 0.3\n"
               "[drive.pedaling]\n"
               "reposition_on_stall = false\nreposition_min_effort_nm = 20.0\n"
               "reposition_stall_window_s = 1.0\nreposition_stall_progress_rad = 0.5\n"
               "reposition_cooldown_s = 1.0\nreposition_back_rate_rad_s = 3.0\n"
               "reposition_timeout_s = 3.0\nreposition_phase_tolerance_rad = 0.05\n"
               "reposition_noop_rad = 0.12\nstop_time_s = 0.35\n"
               "[drive.shifting]\nshift_cooldown_s = 0.4\nshift_cut_duration_s = 0.2\n"
               "[articulated]\nbalance_dwell_s = 0.5\nbalance_grace_s = 3.0\n"
               "joint_kp_nm_rad = 600.0\njoint_kd_nms_rad = 15.0\nleg_active_damping_nms_rad = 35.0\n"
               "[research]\ncontrol_period_s = 0.01\nactuator_delay_s = 0.005\n"
               "sensor_latency_s = 0.01\nsensor_maximum_age_s = 0.1\n"
               "sample_period_s = 0.005\nrecord_decimation = 80\n"
               "[tires]\n"
               "backend = \"pneumatic\"\n"
               "surface_mode = \"track\"\n"
               "[tires.pneumatic]\n"
               "tier = \"fast\"\n";
    }
    const auto physics_arg = physics.string();
    const std::vector<std::string_view> args{
        "--physics-config", physics_arg.c_str(), "--track", "flat",
        "--rider", "lumped",
        "--headless", "--duration", "0.5", "--no-plots",
        "--check-policy", "diagnostic"};
    auto resolved = bike::cli::resolve_ride(
        bike::cli::parse_arguments("bike-ride", args));
    auto prepared = bike::app::prepare(
        std::make_unique<bike::initialization::SetupRequest>(resolved->setup));
    const auto metadata =
        bike::cli::launch_metadata(resolved->setup, *prepared, std::nullopt);
    // The oracle's equilibrium envelope reads land on the launch metadata —
    // residual_qacc and both static loads come from the settled pose the
    // pinned state file supplies.
    const auto &equilibrium = member(metadata, "equilibrium");
    bt::require(number(member(equilibrium, "residual_qacc")) <= .05,
                "the settled residual is inside the envelope");
    bt::require(number(member(equilibrium, "static_front_load_n")) > 0. &&
                    number(member(equilibrium, "static_rear_load_n")) > 0.,
                "both wheels carry static load");
    auto runtime = bike::app::make_ride(std::move(prepared));
    const auto result = runtime->advance(
        400, runtime::RideControl{}, 0., 0., std::nullopt);
    bt::require(result.step == 400,
                "the pneumatic ride commits 400 steps end to end");
    // runtime.sample.channels' twin: the live latest sample — the ride
    // profile keeps retention off, exactly like the oracle's unrecorded
    // runtime.sample view.
    const auto &latest = runtime->snapshot().latest_sample;
    bt::require(latest != nullptr, "the ride publishes a live sample");
    const auto wire = latest->as_wire();
    const auto &tires = wire_obj(wire, "tires");
    for (const char *side : {"front", "rear"}) {
        const auto &entry = wire_obj(tires, side);
        bt::require(sw::string(sw::required(entry, "backend")) ==
                        "pneumatic",
                    "the tire channel names the pneumatic backend");
        bt::require(sw::number(sw::required(entry, "normal_load_n")) > 0.,
                    "the wheel carries normal load");
        bt::require(!sw::array(sw::required(entry, "patches")).empty(),
                    "the tire holds at least one patch");
    }
    // energy['elastic_energy_j']['tires'] > 0 — the carcass stores energy.
    const auto &energy = wire_obj(wire, "energy");
    const auto &elastic = wire_obj(energy, "elastic_energy_j");
    bt::require(sw::number(sw::required(elastic, "tires")) > 0.,
                "the carcass stores positive elastic energy");
    runtime->close();
}

} // namespace

int main() {
    return bt::run([] {
        test_pneumatic_backend_runs_a_physical_ride_end_to_end();
    });
}
