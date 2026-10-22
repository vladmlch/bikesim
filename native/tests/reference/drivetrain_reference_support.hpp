#pragma once
#include "policy_reference_support.hpp"
#include "support/model_fixture.hpp"
#include "initialization/capture.hpp"
#include "model/mapping.hpp"
#include "stepper.hpp"

namespace drivetrain_reference {
using bike::io::Value;
using namespace runtime::setupvalue;
namespace cfg = bike::configuration;
inline constexpr std::array<std::string_view, 3> kinds{
    "elastic_chain", "ideal_mid_drive", "geometric_ideal_mid_drive"};

// Verbatim model_xml() topology, unrelated carrier joint limit and dense /
// sparse Jacobian variants from test_native_drivetrain.py.
inline std::string model_xml(std::string_view kind, std::string_view mode = "crank_effort", // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
                             std::string_view topology = "plain", bool sparse = false) {
    std::vector<std::string> names{"root_x", "frame_pitch", "crank_spin", "rear_carrier", "rear_wheel_spin",
        "front_wheel_spin", "pedal_front_spin", "pedal_rear_spin"};
    std::string extra;
    if (kind == "elastic_chain") {
        extra += R"(<body name="cassette" pos="-.5 0 0"><joint name="cassette_spin" axis="0 1 0"/><geom size=".04" mass="1"/></body>)";
        names.emplace_back("cassette_spin");
    }
    if (topology == "clutch") {
        extra += R"(<body name="drive_shaft"><joint name="drive_shaft_spin" axis="0 1 0"/><geom size=".04" mass="1"/></body>)";
        names.emplace_back("drive_shaft_spin");
    }
    if (topology == "rotor") {
        extra += R"(<body name="rotor"><joint name="rotor_spin" axis="0 1 0"/><inertial pos="0 0 0" mass="1" diaginertia=".2 .2 .2"/></body>)";
        names.emplace_back("rotor_spin");
    }
    const std::string driver = topology == "clutch" ? "drive_shaft_spin" : "crank_spin";
    std::string tendon;
    if (kind == "ideal_mid_drive")
        tendon = R"(<fixed name="ideal_mid_drive_freehub" limited="true" range="-100 0"><joint joint=")" +
            driver + R"(" coef="1.4166666666666667"/><joint joint="rear_wheel_spin" coef="-1"/></fixed>)";
    else if (kind == "geometric_ideal_mid_drive") {
        tendon = R"(<fixed name="geometric_mid_drive_freehub" limited="true" range="-100 0">)";
        for (const auto& name : names) tendon += "<joint joint=\"" + name + R"(" coef=".1"/>)";
        tendon += "</fixed>";
    }
    if (topology == "clutch") tendon += R"(<fixed name="crank_clutch" limited="true" range="-100 0"><joint joint="crank_spin" coef="1"/><joint joint="drive_shaft_spin" coef="-1"/></fixed>)";
    if (topology == "rotor") tendon += R"(<fixed name="motor_freewheel" limited="true" range="-100 0"><joint joint="rotor_spin" coef="1"/><joint joint="crank_spin" coef="-1"/></fixed>)";
    const std::string human = mode == "crank_effort" ? R"(<motor name="human_crank" joint="crank_spin"/>)" : "";
    const std::string motor = topology == "rotor" ? "rotor_spin" : driver;
    return R"(<mujoco><option timestep=".0002" gravity="0 0 0" jacobian=")" + std::string(sparse ? "sparse" : "dense") + R"("/>
    <default><geom type="sphere" size=".05" mass="1" contype="0" conaffinity="0"/><joint damping="0"/></default>
    <worldbody><body name="frame"><joint name="root_x" type="slide" axis="1 0 0"/><joint name="frame_pitch" axis="0 1 0"/><geom/>
      <body name="crank"><joint name="crank_spin" axis="0 1 0"/><geom/></body>
      <body name="rear_wheel" pos="-.5 0 .1"><joint name="rear_carrier" type="slide" axis="0 0 1" limited="true" range="-.01 .01"/><joint name="rear_wheel_spin" axis="0 1 0"/><geom/></body>
      <body name="front_wheel" pos=".6 0 0"><joint name="front_wheel_spin" axis="0 1 0"/><geom/></body>
      <body name="pedal_front"><joint name="pedal_front_spin" axis="0 1 0"/><geom/></body>
      <body name="pedal_rear"><joint name="pedal_rear_spin" axis="0 1 0"/><geom/></body>)" + extra +
        "</body></worldbody><tendon>" + tendon + "</tendon><actuator>" + human +
        R"(<motor name="mid_drive" joint=")" + motor + "\"/></actuator></mujoco>";
}
inline cfg::PhysicalDriveConfig source(std::string_view kind = "ideal_mid_drive", // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
                                       std::string_view topology = "plain") {
    cfg::PhysicalDriveConfig result; result.transmission_model = kind; result.human_torque_nm = 20.;
    result.motor_clutch = topology == "clutch"; result.rotor_inertia_kgm2 = topology == "rotor" ? .2 : 0.;
    return result;
}
inline Value writer_config(const cfg::PhysicalDriveConfig& source, std::string_view mode = "crank_effort") {
    auto physics = std::make_unique<cfg::SimulationPhysicsConfig>();
    physics->physics_mode = "physical"; physics->drive_mode = mode; physics->drive = source;
    auto drive = runtime::project_drive_config(*physics);
    return Value{Value::Object{{"schema", integer(2)}, {"drive", std::move(drive)}}};
}
inline std::unique_ptr<Stepper> bare(std::string_view xml) {
    bt::ModelFixture const fixture(xml);
    const auto image = bike::initialization::capture_model_image(*fixture.model);
    return std::make_unique<Stepper>(image.mjb);
}
inline std::unique_ptr<Stepper> pair(const cfg::PhysicalDriveConfig& source,
    std::string_view mode = "crank_effort", std::string_view topology = "plain", bool sparse = false) {
    bt::ModelFixture fixture(model_xml(source.transmission_model, mode, topology, sparse));
    const auto position = model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq);
    const auto velocity = model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv);
    position[static_cast<std::size_t>(bike::model::joint_address(*fixture.model, "rear_carrier").first)] = .013;
    velocity[static_cast<std::size_t>(fixture.dof("crank_spin"))] = 4.;
    velocity[static_cast<std::size_t>(fixture.dof("rear_wheel_spin"))] = 5.;
    if (topology != "plain") velocity[static_cast<std::size_t>(fixture.dof(topology == "rotor" ? "rotor_spin" : "drive_shaft_spin"))] = 4.5;
    engine::forward(fixture.model.get(), fixture.data.get());
    const auto image = bike::initialization::capture_model_image(*fixture.model);
    auto result = std::make_unique<Stepper>(image.mjb, nativecfg::decode(writer_config(source, mode)));
    result->set_state(position, velocity,
        model_access::readonly_buffer(fixture.data->act, fixture.model->na),
        model_access::readonly_buffer(fixture.data->qacc_warmstart, fixture.model->nv), fixture.data->time);
    result->forward(); result->drive().reset(); return result;
}
inline std::unique_ptr<Stepper> pair(std::string_view kind = "ideal_mid_drive", std::string_view mode = "crank_effort",
                                     std::string_view topology = "plain", bool sparse = false) {
    return pair(source(kind, topology), mode, topology, sparse);
}
inline bt::oracle::Node node(std::string_view function, std::string_view parameters = {}) { // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API.
    std::string name = "tests/reference/test_native_drivetrain.py::" + std::string(function);
    if (!parameters.empty()) name += "[" + std::string(parameters) + "]";
    return bt::oracle::Node(name);
}
inline void array(bt::oracle::Node& golden, std::span<const double> actual, std::string_view label = "") {
    const std::array shape{actual.size()}; golden.f64(actual, shape, label);
}
inline void scalar(bt::oracle::Node& golden, double actual, std::string_view label = "") {
    policy_reference::scalar(golden, actual, label);
}
// A DriveSnapshot's range/coefficients/prepared buffers are ndarrays in
// Python; its angles and policy/config lists remain scalar sequences.
// NOLINTNEXTLINE(misc-no-recursion) recursive JSON value tree walk
inline void tree(bt::oracle::Node& golden, const Value& value, const std::string& path = "$") {
    if (std::holds_alternative<Value::Object>(value.data)) {
        policy_reference::keys(golden, value, path);
        for (const auto& key : policy_reference::sorted_keys(value)) { std::string next(path); next += '.'; next += key; tree(golden, field(value, key), next); }
    } else if (const auto* values = std::get_if<Value::Array>(&value.data)) {
        if (path.ends_with(".range") || path.ends_with(".coefficients") || path.ends_with(".jacobian") || path.ends_with(".qpos")) {
            std::vector<double> decoded; decoded.reserve(values->size());
            for (const auto& element : *values) decoded.push_back(bike::io::as_double(element, path));
            array(golden, decoded, path);
        } else {
            policy_reference::length(golden, values->size(), path);
            for (std::size_t i = 0; i < values->size(); ++i) tree(golden, (*values)[i], path + "[" + std::to_string(i) + "]");
        }
    } else policy_reference::tree(golden, value, path);
}
inline Value state(const Stepper& owner) { return drivetrain::encode_drive_snapshot(owner.drive().state()); }
inline void state(bt::oracle::Node& golden, const Stepper& owner) { tree(golden, state(owner)); }
inline void restore(Stepper const& owner, const Value& value) { owner.drive().restore(drivetrain::decode_drive_snapshot(value)); }
inline Value diagnostics(const Stepper& owner, bool probe = false) {
    return field(state(owner), probe ? "probe_last" : "last");
}
inline Value stored_energy(Stepper const& owner) {
    Value::Object result;
    for (const auto& [key, value] : owner.drive().stored_energy()) std::visit([&](const auto& item) {
        using T = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<T, double>) result.emplace_back(key, real(item));
        else if constexpr (std::is_same_v<T, int>) result.emplace_back(key, integer(item));
        else if constexpr (std::is_same_v<T, bool>) result.emplace_back(key, Value{item});
        else if constexpr (std::is_same_v<T, std::string>) result.emplace_back(key, text(item));
        else result.emplace_back(key, Value{});
    }, value);
    return Value{std::move(result)};
}
inline drivetrain::ForceComponents components(Stepper const& owner, double dt, double speed,
    drivetrain::RideControl control = {}, bool active = true, bool braking = false,
    bool advance = true, double sensed = 0., std::optional<drivetrain::PedalingState> pedal = {}) {
    return owner.drive().components(control, dt, speed, braking, active, advance, sensed, pedal, true, std::nullopt);
}
inline void components(bt::oracle::Node& golden, const drivetrain::ForceComponents& actual) {
    std::vector<std::string> names; for (const auto& [name, ignored] : actual) { (void)ignored; names.push_back(name); }
    std::ranges::sort(names);
    Value::Array keys; for (const auto& name : names) keys.push_back(text(name));
    policy_reference::json_bytes(golden, Value{std::move(keys)}, "$#keys");
    for (const auto& name : names) {
        const auto found = std::ranges::find(actual, name, &drivetrain::ForceComponents::value_type::first);
        bt::require(found != actual.end(), "each force component has its source name");
        array(golden, found->second, "$." + name);
    }
}
inline std::vector<double> total(const Stepper& owner, const drivetrain::ForceComponents& values) {
    std::vector<double> result(static_cast<std::size_t>(owner.model()->nv));
    for (const auto& [name, row] : values) {
        (void)name; bt::require(row.size() == result.size(), "source component width matches the engine");
        for (std::size_t i = 0; i < result.size(); ++i) result[i] += row[i];
    }
    return result;
}
inline void step(Stepper& owner, const drivetrain::ForceComponents& values) {
    owner.set_inputs(owner.ctrl(), total(owner, values)); owner.step();
}
inline void position(Stepper const& owner, std::string_view joint, double value) {
    const auto address = bike::model::joint_address(*owner.model(), joint).first;
    model_access::mutable_buffer(owner.data()->qpos, owner.model()->nq)[static_cast<std::size_t>(address)] = value;
}
inline void velocity(Stepper const& owner, std::string_view joint, double value) {
    const auto address = bike::model::joint_address(*owner.model(), joint).second;
    model_access::mutable_buffer(owner.data()->qvel, owner.model()->nv)[static_cast<std::size_t>(address)] = value;
}
inline void forward(Stepper& owner) { owner.note_posvel_stale(); owner.forward(); }
inline void reset_engine_state(Stepper& owner) {
    owner.set_state(owner.qpos(), owner.qvel(),
        model_access::readonly_buffer(owner.data()->act, owner.model()->na),
        model_access::readonly_buffer(owner.data()->qacc_warmstart, owner.model()->nv), owner.time());
    owner.forward();
}
inline void clear_pending(Stepper const& owner) {
    auto saved = std::make_unique<drivetrain::DriveSnapshot>(owner.drive().state());
    saved->pending_actuation.reset(); owner.drive().restore(*saved);
}
inline std::array<std::uint64_t, 4> storage(const Stepper& owner) {
    const auto* transmission = owner.drive().transmission_storage();
    bt::require(transmission != nullptr, "geometric owner has a transmission");
    const auto* prepared = transmission->prepared_storage();
    bt::require(prepared != nullptr, "geometric owner has persistent prepared buffers");
    const auto generations = transmission->prepared_generations();
    return {generations.first, generations.second, static_cast<std::uint64_t>(prepared->jacobian.capacity()),
        static_cast<std::uint64_t>(prepared->qpos.capacity())};
}
template<class Exception, class F> void error(F&& action, std::string_view message = {}) {
    bool rejected = false;
    try { std::forward<F>(action)(); } catch (const Exception& failure) {
        rejected = true;
        bt::require(message.empty() || std::string_view(failure.what()).find(message) != std::string_view::npos,
            "native exception retains the source diagnostic");
    }
    bt::require(rejected, "native operation rejects the original invalid request");
}
} // namespace drivetrain_reference
