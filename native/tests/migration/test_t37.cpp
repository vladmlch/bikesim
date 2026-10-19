// T37 source-derived quality, status, immutable-summary and retention checks.
// Authored without compilation or execution; this is not oracle evidence.
#include "support/migration_test.hpp"
#include "diag.hpp"
#include "research/evidence.hpp"
#include "research/policy_session.hpp"
#include "io/hash.hpp"
#include "io/json.hpp"
#include "runtime/setup_values.hpp"
#include <algorithm>
#include <bit>
#include <limits>

namespace {
using namespace bike::research;
namespace wire = runtime::sample_wire;
namespace value = runtime::setupvalue;
bool flag(const bike::io::Value& object, std::string_view key) {
    return bike::io::as_bool(value::field(object, key), key);
}
void same_bits(std::span<const double> first, std::span<const double> second, std::string_view why) {
    bt::require(first.size() == second.size(), why);
    for (std::size_t i = 0; i < first.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(first[i]) == std::bit_cast<std::uint64_t>(second[i]), why);
}
runtime::WireObject valid_channels() {
    const runtime::WireObject tire{{"unloaded_radius_m", .35}, {"normal_load_n", 100.}, {"penetration_m", 0.}};
    return {{"tires", runtime::WireObject{{"front", tire}, {"rear", tire}}},
        {"suspension", runtime::WireObject{{"linkage_closure_max_m", 0.}}},
        {"energy", runtime::WireObject{{"energy_scale_j", 100.}, {"residual_j", 0.}}}};
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")

[[gnu::noinline]] void energy_contract() {
    runtime::WireObject energy{{"energy_scale_j", 50.}, {"external_work_j", -25.},
        {"source_positive_work_j", 25.}, {"active_work_j", 1e30}, {"residual_j", -4.},
        {"material_loss_j", 1e30}, {"constraint_work_j", 1e30},
        {"electrical_work_j", 20.}, {"electrical_residual_j", 0.}};
    const auto quality = energy_quality(energy);
    bt::require(quality.residual_ratio == .04 && quality.acceptable,
        "initial, external and positive source work define the energy denominator");
    wire::set(energy, "source_positive_work_j", 0.);
    bt::require(!energy_quality(energy).acceptable,
        "loss, constraint work and an overridden active-work field cannot hide residuals");
    wire::set(energy, "source_positive_work_j", 25.);
    wire::set(energy, "electrical_residual_j", 1e-7 * 20.);
    bt::require(energy_quality(energy).acceptable, "electrical acceptance includes its exact boundary");
    wire::set(energy, "electrical_residual_j", 1e-7 * 21.);
    bt::require(!energy_quality(energy).acceptable, "electrical rejection is independent of the mechanical ratio");
    bt::invalid([&] { (void)energy_quality(energy, 0.); });
    bt::invalid([&] { (void)energy_quality(energy, std::numeric_limits<double>::infinity()); });
    wire::set(energy, "residual_j", std::numeric_limits<double>::quiet_NaN());
    bt::invalid([&] { (void)energy_quality(energy); });
    bt::invalid([] { (void)energy_quality({{"energy_scale_j", -1.}}); });
    bool zero_rejected = false;
    try { (void)energy_quality({{"energy_scale_j", 0.}}); }
    catch (const std::runtime_error&) { zero_rejected = true; }
    bt::require(zero_rejected && energy_quality({}).acceptable, "explicit zero scale is not silently floored");
}
[[gnu::noinline]] void status_contract() {
    auto channels = valid_channels();
    const runtime::WireObject front{{"unloaded_radius_m", .35}, {"normal_load_n", 100.}, {"penetration_m", .1},
        {"multi_support", true}, {"outside_material_load_range", true}, {"outside_material_deflection_range", true},
        {"patches", runtime::WireArray{runtime::WireObject{{"source_geom", "catch_plane"}, {"normal_load_n", 1.}}}},
        {"outside_profile_domain", true}};
    wire::set(channels, "tires", runtime::WireObject{{"front", front}, {"rear", runtime::WireObject{}}});
    wire::set(channels, "suspension", runtime::WireObject{{"linkage_closure_max_m", .003}});
    wire::set(channels, "attachment_violations", runtime::WireArray{"support:overload"});
    wire::set(channels, "energy", runtime::WireObject{{"energy_scale_j", 100.}, {"residual_j", 100.}});
    const std::vector<std::string> expected{"front:multi_support", "front:material_load_range",
        "front:material_deflection_range", "front:tire_compression", "front:catch_plane",
        "front:profile_domain", "rear:missing_radius", "linkage:closure_error", "support:overload"};
    runtime::PhysicalSampleData sample; sample.channels = channels;
    bt::require(model_violations(sample) == expected, "model checks retain source ordering and measured support violations");
    bt::invalid([&] { (void)model_violations(sample, -.1); });
    runtime::ModelStatus status;
    status.observe(0, 0., channels);
    const auto first = status.as_wire();
    status.observe(1, .001, valid_channels());
    bt::require(!wire::boolean(status.numerically_valid) && !wire::boolean(wire::required(status.as_wire(), "model_valid")),
        "numeric and applicability failures stay latched after a valid interval");
    bt::require(bike::io::canonical_json(wire_value(wire::required(first, "first_model_violation"))) ==
        bike::io::canonical_json(wire_value(wire::required(status.as_wire(), "first_model_violation"))),
        "the first violation is preserved by value");
    bt::invalid([&] { status.observe(1, .001, valid_channels()); });
    status.maximum_compression_fraction = .2; status.calibration_status = "declared_unvalidated";
    status.reset();
    bt::require(status.counts.empty() && !status.first && status.last_interval == -1 &&
        wire::string(status.numerically_valid) == "not_evaluated" && status.maximum_compression_fraction == .2 &&
        status.calibration_status == "declared_unvalidated", "status reset retains configuration and calibration identity");
    auto empty_energy = valid_channels(); wire::set(empty_energy, "energy", runtime::WireObject{});
    status.observe(0, 0., empty_energy);
    status.observe(1, .001, channels, false);
    bt::require(wire::string(status.numerically_valid) == "not_evaluated", "empty or unmeasured energy does not evaluate numerics");
    status.observe(2, .002, valid_channels());
    bt::require(wire::boolean(status.numerically_valid), "first measured energy closes numerical quality");
    runtime::ReferenceMonitor monitor;
    monitor.observe({.time_s=0., .violations=expected}); monitor.observe({.time_s=1., .violations={"later"}});
    bt::require(monitor.take_warning().has_value() && !monitor.take_warning() && monitor.first_failure.value().time_s == 0., // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
        "diagnostic reference monitor retains and reports its first rejection once");
    for (const bool numeric : {false, true})
        for (const bool model : {false, true}) {
            runtime::ResearchTransition transition; transition.numerically_valid = numeric; transition.model_valid = model;
            bt::require(transition.valid_for_learning() == (numeric && model), "transition eligibility requires both validity axes");
            for (const auto reason : {"simulation_error", "policy_error"}) {
                transition.reason = reason;
                bt::require(!transition.valid_for_learning(), "failure reasons reject a transition independently of validity");
            }
        }
}
[[gnu::noinline]] void rider_sensor_boundary() {
    const auto sample = std::make_shared<runtime::PhysicalSampleData>();
    sample->channels = {{"sensors", runtime::WireObject{
        {"frame_specific_force_body_mps2", runtime::WireArray{1., 2., 3.}},
        {"frame_gyro_body_rad_s", runtime::WireArray{4., -5., 6.}}}},
        {"rider", runtime::WireObject{{"saddle", runtime::WireObject{{"normal_load_n", 20.}}},
            {"front_pedal", runtime::WireObject{{"normal_load_n", 60.}}},
            {"rear_pedal", runtime::WireObject{{"normal_load_n", 40.}}},
            {"grip", runtime::WireObject{{"force_on_bike_n", runtime::WireArray{3., 4., 0.}}}}}}};
    const auto signals = signals_from_sample(sample);
    bt::require(signals.pitch_rate_up_rad_s == 5. && signals.specific_force_body_mps2[2] == 3. &&
        signals.saddle_load_n == 20. && signals.pedal_load_n == 100. && signals.bar_load_n == 5.,
        "rider behavior receives proprioception rather than road/contact truth");
    bt::require(signals_from_sample({}).pedal_load_n == 0., "startup has no invented prior rider sample");
    auto observation = runtime::SensorObservation{}.as_wire(); wire::set(observation, "truth", runtime::WireObject{});
    bt::invalid([&] { (void)runtime::SensorObservation::from_wire(observation); });
}
[[gnu::noinline]] void owned_evidence() {
    auto request = std::make_unique<ResearchRequest>(read_research_request(bt::fixture("requests/research_delay.toml")));
    request->capture_mode = runtime::ResearchCaptureMode::full;
    request->experiment.duration_s = .02;
    request->experiment.stop_on_model_violation = false;
    request->experiment.maximum_energy_residual_ratio = 1e12;
    request->setup.model.physics.value().initial_speed_mps = 0.; // NOLINT(bugprone-unchecked-optional-access) fixture contract guarantees engagement; .value() is the loud check
    auto seed = std::make_unique<PreparedEnvironment>(prepare_environment(std::move(*request)));
    PolicySession full(runtime::make_research(seed->bootstrap), load_policy("zero"));
    auto off_seed = std::make_unique<runtime::ResearchBootstrap>(seed->bootstrap);
    off_seed->config.capture_mode = runtime::ResearchCaptureMode::off;
    off_seed->config.assets.reset(); off_seed->ride.config.record_retention = runtime::RetentionMode::off;
    PolicySession off(runtime::make_research(std::move(*off_seed)), load_policy("zero"));
    (void)full.advance(); (void)off.advance(); full.request_stop(); off.request_stop();
    auto captured = std::make_unique<runtime::OwnedResearchEpisode>(full.seal_episode());
    auto unrecorded = std::make_unique<runtime::OwnedResearchEpisode>(off.seal_episode());
    same_bits(captured->final_snapshot.integration_state, unrecorded->final_snapshot.integration_state,
        "optional retention does not change physical integration");
    for (const auto key : {"metrics", "accounting", "requested_count", "completed_count", "torque_delivered_nms"})
        bt::require(bike::io::canonical_json(wire_value(wire::required(captured->evaluation, key))) ==
            bike::io::canonical_json(wire_value(wire::required(unrecorded->evaluation, key))),
            "live metrics and accounting do not depend on optional histories");
    bt::require(unrecorded->physical.samples.empty() && unrecorded->requested.empty() && unrecorded->applied.empty() &&
        unrecorded->observations.empty() && unrecorded->transitions.empty(), "off capture retains no optional trajectory rows");
    const auto summary = summarize(*captured);
    const auto& extents = value::field(summary, "recording_extents");
    bt::require(bike::io::as_int64(value::field(extents, "physical_steps"), "physical_steps") == captured->final_snapshot.step &&
        bike::io::as_int64(value::field(extents, "accounted_steps"), "accounted_steps") == captured->final_snapshot.step,
        "summary keeps physical, accounting and control extents distinct");
    bt::require(captured->requested.size() == 1 && captured->transitions.size() == 1 && captured->observations.size() == 2,
        "complete control evidence includes its preceding and resulting observations");
    const auto recorded = recording_batch(*captured, "transitions", 0, 1);
    bt::require(wire::integer(wire::required(wire::object(recorded.front()), "request_id")) == 0 &&
        wire::integer(wire::required(wire::object(recorded.front()), "observation_id")) == 1 &&
        !wire::find(wire::object(captured->batch("transitions", 0, 1).front()), "request_id"),
        "recording adds stable IDs without changing the frozen T10 batch schema");
    bt::invalid([&] { (void)recording_batch(*captured, "transitions", -1, 1); });
    const auto before = bike::io::canonical_json(summary);
    const auto assets = captured->configuration.assets;
    bt::require(assets && !assets->empty(), "recorded episode owns immutable rebuild assets");
    for (const auto& asset : *assets) bt::require(bike::io::sha256(asset.content) == asset.sha256, "captured asset hash matches its owned bytes");
    full.reset("184467440737095516160001");
    const auto reset = full.environment().configuration();
    bt::require(bike::io::canonical_json(value::field(value::field(reset.metadata, "resolved_config"), "seed")) == reset.seed_decimal &&
        bike::io::canonical_json(value::field(value::field(reset.metadata, "experiment"), "seed")) == seed->bootstrap.config.seed_decimal,
        "actual reset seed is separate from the original experiment request");
    bt::require(reset.assets != assets && captured->configuration.assets == assets, "reset gives the new recipe an independent immutable owner");
    full.close(); off.close();
    bt::require(bike::io::canonical_json(summarize(*captured)) == before,
        "summary and retained samples remain readable after reset and engine destruction");
    auto invalid = std::make_unique<runtime::OwnedResearchEpisode>(*captured);
    wire::set(invalid->evaluation, "model_valid", false);
    bt::require(!flag(value::field(summarize(*invalid), "research"), "valid_for_learning"), "model violation blocks learning eligibility");
    value::set(invalid->run_metadata, "seal_reason", value::text("simulation_error"));
    bt::require(!flag(value::field(summarize(*invalid), "recording_extents"), "replay_eligible"), "failure seals cannot qualify for replay");
    value::set(invalid->run_metadata, "seal_reason", value::text("operator_stop"));
    value::set(invalid->run_metadata, "identity_checks", bike::io::Value{bike::io::Value::Object{}});
    bt::require(!flag(value::field(summarize(*invalid), "recording_extents"), "replay_eligible"), "missing identity qualification fails closed");
    std::erase_if(invalid->evaluation, [](const auto& member) { return member.first == "metrics"; });
    bt::invalid([&] { (void)summarize(*invalid); });
}
} // namespace
int main() { return bt::run([] { energy_contract(); status_contract(); rider_sensor_boundary(); owned_evidence(); }); }
NATIVE_DIAG_POP
