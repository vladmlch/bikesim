// Authored T26/T30 integration regression; not executed during this wave.
#include "support/migration_test.hpp"
#include "drivetrain/transmission.hpp"
#include "engine_call.hpp"
#include "model/builder.hpp"
#include "runtime/control.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <ranges>
#include <vector>

namespace {
void native_rows_and_bridge() {
    constexpr auto xml = R"xml(
<mujoco model="native_contact_contract">
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="terrain" type="plane" size="10 10 0.1"/>
    <geom name="catch_plane" type="plane" pos="0 0 -10" size="10 10 0.1"/>
    <geom name="geom_handlebar" type="sphere" pos="0 0 2" size="0.05"/>
    <body name="front" pos="1 0 0.45">
      <joint name="front_x" type="slide" axis="1 0 0"/>
      <joint name="front_z" type="slide" axis="0 0 1"/>
      <geom name="geom_front_contact" type="sphere" size="0.5" mass="1"/>
    </body>
    <body name="rear" pos="-1 0 0.8">
      <joint name="rear_z" type="slide" axis="0 0 1"/>
      <geom name="geom_rear_contact" type="sphere" size="0.5" mass="1"/>
    </body>
  </worldbody>
</mujoco>)xml";
    auto model = bike::model::compile_mjcf(xml);
    drivetrain::OwnedData data(engine::make_data(model.get()));
    engine::forward(model.get(), data.get());
    runtime::RuntimeContactQuery query(model.get());
    std::vector<double> incoming(static_cast<std::size_t>(model->nv), 0.);
    incoming.front() = 2.;
    const auto first = query.query(model.get(), data.get(), 0., incoming);
    bt::require(first.contacts.front_load_n > 1. && first.contacts.rear_load_n == 0.,
        "engine sphere rows supply current raw load for the touching wheel");
    bt::require(first.snapshots[0].backend == "native_reference" &&
                first.snapshots[0].geometric_contact && !first.snapshots[1].geometric_contact,
        "native contact snapshot declares its actual backend and geometry");
    bt::require(first.contacts.front_controller_grounded.value_or(false),
        "terrain load reaches the independent controller contact filter");
    bt::require(!first.snapshots[0].patches.empty(), "native wheel contact contains at least one solved patch");
    const auto &patch = first.snapshots[0].patches.front();
    bt::require(patch.native_world_force_n.has_value() && patch.working_surface,
        "native patch retains full world wrench and terrain provenance");
    bt::require(std::abs(patch.slip_mps - 2.) < 1e-12,
        "contact slip uses the supplied incoming generalized velocity");
    bt::require(std::abs((patch.native_world_force_n.value())[2] - patch.normal_load_n) < 1e-9, // NOLINT(bugprone-unchecked-optional-access) optional engagement is the fixture/request contract under test; .value() reports violations loudly
        "flat plane normal transforms into an upward wheel force");
    const auto state = query.state();
    bt::invalid([&] {
        (void)query.query(model.get(), data.get(), std::numeric_limits<double>::quiet_NaN());
    });
    bt::require(query.state().front_load.held_load_n == state.front_load.held_load_n &&
                query.state().front_controller.last_time_s == state.front_controller.last_time_s,
        "invalid contact input does not advance a bridge or clock");
    bt::invalid([&] { (void)query.query(model.get(), data.get(), 0., std::array{1.}); });

    const int front_z = mj_name2id(model.get(), mjOBJ_JOINT, "front_z");
    const auto addresses = std::views::counted(model->jnt_qposadr, model->njnt);
    auto const qpos = std::views::counted(data->qpos, model->nq);
    qpos[static_cast<std::size_t>(addresses[static_cast<std::size_t>(front_z)])] = 1.;
    engine::forward(model.get(), data.get());
    const auto dropout = query.query(model.get(), data.get(), .001);
    bt::require(dropout.contacts.front_load_n == first.contacts.front_load_n &&
                dropout.contacts.front_support_n == 0. && dropout.snapshots[0].patches.empty(),
        "legacy magnitude bridge does not fabricate current support or contact patches");
    runtime::NativeContactResult expired;
    for (int step = 2; step <= 11; ++step)
        expired = query.query(model.get(), data.get(), static_cast<double>(step) * .001);
    bt::require(expired.contacts.front_load_n == 0. &&
                !expired.contacts.front_controller_grounded.value_or(true),
        "dropout counters and grounded duration expire independently");
    auto other_model = bike::model::compile_mjcf(xml);
    bt::invalid([&] { (void)query.query(other_model.get(), data.get()); });
    bt::invalid([&] { (void)query.handlebar_load(other_model.get(), data.get()); });
    bt::invalid([&] { (void)query.handlebar_load(model.get(), nullptr); });
}
} // namespace

int main() { return bt::run(native_rows_and_bridge); }
