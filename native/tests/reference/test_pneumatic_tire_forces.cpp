// Fifteen property/contract functions from test_pneumatic_tire_forces.py.
// The tier-parametrized frozen oracle comparison lives in the sibling
// test_pneumatic_tire_oracle.cpp, and cannot pass without platform captures.
#include "pneumatic_tire_fixture.hpp"
#include "configuration/load.hpp"
#include "io/json.hpp"
#include "interval_clock.hpp"
#include "runtime/control.hpp"

#include <set>
#include <type_traits>
#include <utility>
#include <variant>

namespace {
using namespace bt::pneumatic;
[[gnu::noinline]] void test_constructor_rejects_wrong_backend_and_missing_surface() {
    // test_pneumatic_tire_forces.py::test_constructor_rejects_wrong_backend_and_missing_surface
    Rig rig;
    auto cfg = std::make_unique<nativecfg::TireConfig>(configuration());
    cfg->backend = "native_reference";
    // Each rejected writer is a ~4KB object; heap owners keep this frame
    // under the 8192-byte budget. Constructor rejection is unchanged.
    rejects([&] { (void)std::make_unique<PneumaticTireWriter>(rig.engine.model.get(), rig.engine.data.get(), *cfg); }, "config.tire.backend");
    cfg->backend = "pneumatic"; cfg->pneumatic.reset();
    rejects([&] { (void)std::make_unique<PneumaticTireWriter>(rig.engine.model.get(), rig.engine.data.get(), *cfg); }, "config.tire.pneumatic");
    // Python's legacy TyreConfig is a different native type and cannot
    // enter this constructor. The missing-section value control above
    // exercises the resolved-wire boundary that is reachable in C++.
    static_assert(!std::is_constructible_v<PneumaticTireWriter, const mjModel *, const mjData *, bike::legacy::TyreConfig>);
    *cfg = configuration(); cfg->surface_mode = "track"; cfg->pneumatic->surface.reset(); // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
    rejects([&] { (void)std::make_unique<PneumaticTireWriter>(rig.engine.model.get(), rig.engine.data.get(), *cfg); }, "config.tire.surface_map");
}
[[gnu::noinline]] void test_constructor_validates_contact_geom_contype_and_radius() {
    // test_pneumatic_tire_forces.py::test_constructor_validates_contact_geom_contype_and_radius
    Rig rig;
    const auto geom = static_cast<std::size_t>(bike::model::required_id(*rig.engine.model, mjOBJ_GEOM, "geom_rear_contact"));
    const auto construct = [&] { PneumaticTireWriter const bad(rig.engine.model.get(), rig.engine.data.get(), Access::config(*rig.tire)); };
    auto const type = model_access::mutable_buffer(rig.engine.model->geom_contype, rig.engine.model->ngeom);
    auto const affinity = model_access::mutable_buffer(rig.engine.model->geom_conaffinity, rig.engine.model->ngeom);
    type[geom] = 1;
    rejects(construct, "collisions must be disabled");
    type[geom] = 0; affinity[geom] = 1;
    rejects(construct, "collisions must be disabled");
    affinity[geom] = 0;
    model_access::mutable_buffer(rig.engine.model->geom_size, 3 * rig.engine.model->ngeom)[3 * geom] += 1e-4;
    rejects(construct, "does not match");
    rejects([] { Rig const bad("fast", .0005, .004, 5.,
        R"(<geom name="geom_rear_hub" type="cylinder" fromto="0 -0.07 0 0 0.07 0" size="0.02" mass="0"/>)"); }, "collisions must be disabled");
}
[[gnu::noinline]] void test_qfrc_sanity_on_a_physical_model() {
    // test_pneumatic_tire_forces.py::test_qfrc_sanity_on_a_physical_model
    auto request = std::make_unique<bike::model::BuildRequest>();
    request->mode = "ride"; request->include_rider = false;
    request->physics = bike::configuration::detail::construct_config(bike::io::parse_json(
        R"({"physics_mode":"physical","drive_mode":"coast","tires":{"backend":"pneumatic","pneumatic":{"tier":"fast","surface":"asphalt"}}})"));
    bt::ModelFixture model(bike::model::generate_mjcf(*request));
    std::array<int, 2> bodies{};
    for (std::size_t side = 0; side < sides.size(); ++side) {
        bodies[side] = bike::model::required_id(*model.model, mjOBJ_BODY, std::string(sides[side]) + "_wheel");
        const auto of = model_access::readonly_buffer(model.model->geom_bodyid, model.model->ngeom);
        auto const type = model_access::mutable_buffer(model.model->geom_contype, model.model->ngeom);
        auto const affinity = model_access::mutable_buffer(model.model->geom_conaffinity, model.model->ngeom);
        for (std::size_t g = 0; g < of.size(); ++g)
            if (of[g] == bodies[side]) { type[g] = 0; affinity[g] = 0; }
    }
    engine::forward(model.model.get(), model.data.get());
    const auto positions = model_access::readonly_buffer(model.data->xpos, 3 * model.model->nbody);
    const double front = positions[3 * static_cast<std::size_t>(bodies[0])];
    const double rear = positions[3 * static_cast<std::size_t>(bodies[1])];
    const auto x = bike::geometry::detail::np_linspace(std::min(front, rear) - 1., std::max(front, rear) + 1., 2001);
    auto cfg = std::make_unique<nativecfg::TireConfig>(configuration());
    cfg->pneumatic->profile = {.x0_m=x.front(), .dx_m=x[1] - x[0], // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
        .z_m=std::vector<double>(x.size(), bike::geometry::compute_ground_z(request->bike) / 1000.)};
    auto tire = std::make_unique<PneumaticTireWriter>(model.model.get(), model.data.get(), *cfg);
    const auto root_z = bike::model::joint_address(*model.model, "root_z").first;
    model_access::mutable_buffer(model.data->qpos, model.model->nq)[static_cast<std::size_t>(root_z)] -= .015;
    model_access::mutable_buffer(model.data->qvel, model.model->nv)[static_cast<std::size_t>(model.dof("rear_wheel_spin"))] = 3.;
    engine::forward(model.model.get(), model.data.get());
    const auto qfrc = tire->qfrc(model.data.get(), model.model->opt.timestep);
    bt::require(qfrc.size() == static_cast<std::size_t>(model.model->nv), "physical qfrc width");
    bt::require(std::ranges::all_of(qfrc, [](double x0) { return std::isfinite(x0); }), "physical qfrc finite");
    bt::require(std::ranges::any_of(qfrc, [](double x0) { return x0 != 0.; }), "physical qfrc nonzero");
    std::vector<double> expected(qfrc.size(), 0.);
    for (std::size_t side = 0; side < sides.size(); ++side) {
        const auto &snapshot = tire->snapshot_storage()[side];
        bt::require(tire->diagnostic_storage()[side].normal_load_n > 0. && !snapshot.patches.empty(), "both physical tyres loaded");
        bt::require(snapshot.geometric_contact && snapshot.backend == "pneumatic", "physical pneumatic contact");
        for (const auto &patch : snapshot.patches)
            add_jtf(*model.model, *model.data, sides[side], patch.point_m, runtime::patch_world_force(patch), expected);
    }
    for (std::size_t i = 0; i < qfrc.size(); ++i)
        bt::require(std::abs(qfrc[i] - expected[i]) <= 1e-8 + 1e-9 * std::abs(expected[i]), "physical per-patch J^T F");
}
[[gnu::noinline]] void test_snapshot_schema_and_signed_slip() {
    // test_pneumatic_tire_forces.py::test_snapshot_schema_and_signed_slip
    Rig rig; (void)rig.advance();
    const auto &front = rig.tire->snapshot_storage()[0], &rear = rig.tire->snapshot_storage()[1];
    bt::require(front.backend == "pneumatic" && front.geometric_contact && !front.patches.empty(), "typed front pneumatic snapshot");
    bt::require(front.interval_id == interval_clock::interval_id(0., rig.dt()) && front.interval_id == 0, "snapshot interval zero");
    const auto output = Access::wheel(*rig.tire, 0).last_outputs().patches();
    bt::require(front.patches.size() == output.size(), "same snapshot/output patch count");
    for (std::size_t i = 0; i < front.patches.size(); ++i) {
        const auto &patch = front.patches[i];
        bt::require(!patch.native_world_force_n, "pneumatic snapshot has no native force override");
        // The actual published world-force property equals the source's
        // normal/tangent reconstruction, at the original zero tolerance.
        numeric_equal(runtime::patch_world_force(patch), snapshot_force(patch));
        bt::require(patch.slip_mps == output[i].slip_speed_mps && patch.normal_load_n == output[i].normal_load_n,
            "signed slip and normal load preserve output values");
    }
    bt::require(!rear.geometric_contact && rear.patches.empty(), "airborne rear snapshot");
    const auto &diag = rig.tire->diagnostic_storage()[0];
    const auto wire = runtime::wire_pneumatic_tire_diagnostics(diag);
    const std::set<std::string> expected{
        "normal_load_n", "support_n", "tangent_force_n", "slip_mps", "slip_ratio",
        "transient_slip_ratio", "friction_coefficient", "penetration_m", "contact_length_m", "patch_count",
        "multi_support", "supports_multiple_contacts", "fully_sliding", "airborne", "coverage_event",
        "rim_strike_active", "rim_event_x_m", "rim_event_speed_mps", "rim_event_peak_load_n", "rim_event_peak_force_n",
        "rim_event_energy_j", "pressure_bar", "dissipated_power_w", "brush_loss_j", "radial_energy_j", "shear_energy_j",
        "outside_material_load_range", "surface"};
    std::set<std::string> actual;
    for (const auto &[key, value] : wire) {
        actual.insert(key);
        bt::require(!std::holds_alternative<runtime::WireArray>(value.value) &&
            !std::holds_alternative<runtime::WireObject>(value.value), "diagnostic leaves are scalar");
    }
    bt::require(actual == expected && actual.size() == wire.size(), "actual runtime diagnostic key set");
    bt::require(diag.supports_multiple_contacts && !diag.outside_material_load_range &&
        diag.multi_support == (diag.patch_count > 1) && diag.surface == "asphalt", "front diagnostic contract");
    bt::require(rig.tire->diagnostic_storage()[1].surface == "asphalt" && rig.tire->diagnostic_storage()[1].airborne,
        "rear diagnostic material and airborne flag");
}
[[gnu::noinline]] void test_state_wire_names_and_round_trip() {
    // test_pneumatic_tire_forces.py::test_state_wire_names_and_round_trip
    Rig rig;
    const auto n = static_cast<std::size_t>(bike::legacy::tier_spec("fast").n_rays), width = (n + 1) / 2;
    const auto &names = rig.tire->state_names();
    bt::require(names.size() == 2 * (4 * n + 8 + 3 * width), "canonical fast state width");
    const std::array<std::string_view, 4> first{"front.ray.000.delta_prev_m", "front.ray.000.elastic_prev_n", "front.ray.000.maxwell_n", "front.ray.000.bristle_m"};
    for (std::size_t i = 0; i < first.size(); ++i) bt::require(names[i] == first[i], "first four ray cells");
    bt::require(names[4 * n] == "front.carcass.initialized" && names[4 * n + 1] == "front.rim.active" &&
        names[4 * n + 7] == "front.patch.count" && names[4 * n + 8] == "front.patch.00.start" &&
        names.back() == std::format("rear.patch.{:02}.kappa_prime", width - 1), "fixed canonical state positions");
    (void)rig.advance();
    const auto row = rig.tire->state();
    bt::require(row.size() == names.size(), "state row width");
    bt::require(row[column(names, "front.patch.count")] >= 1. &&
        row[column(names, "front.patch.00.stop")] > row[column(names, "front.patch.00.start")], "front patch range populated");
    bt::require(std::isnan(row[column(names, "front.ray.000.bristle_m")]) && row[column(names, "rear.patch.count")] == 0., "fast bristle sentinel and airborne patch count");
    Rig copy; copy.tire->set_state(names, row);
    numeric_equal(copy.tire->state(), row, true);
    numeric_equal(Access::wheel(*copy.tire, 0).carcass_state().delta_prev_m(), Access::wheel(*rig.tire, 0).carcass_state().delta_prev_m());
    bt::require(Access::wheel(*copy.tire, 0).carcass_state().initialized() &&
        patch_keys(Access::wheel(*copy.tire, 0)) == patch_keys(Access::wheel(*rig.tire, 0)), "rebuilt initialized carcass and patch keys");
    rejects<std::runtime_error>([&] { copy.tire->set_state(std::span(names).first(names.size() - 1), std::span(row).first(row.size() - 1)); }, "rear.patch");
    const std::array<std::string, 1> bogus{"front.ray.000.bogus"}; const std::array<double, 1> zero{0.};
    rejects<std::runtime_error>([&] { copy.tire->set_state(bogus, zero); }, "front.ray.000.bogus");
    rejects<std::runtime_error>([&] { copy.tire->set_state(names, std::span(row).first(row.size() - 1)); }, "cells for");
}
[[gnu::noinline]] void test_detailed_tier_bristle_wire() {
    // test_pneumatic_tire_forces.py::test_detailed_tier_bristle_wire
    Rig rig("detailed"); (void)rig.advance();
    const auto &names = rig.tire->state_names(); const auto row = rig.tire->state();
    const auto n = Access::wheel(*rig.tire, 0).n_rays();
    bt::require(names.size() == 2 * (4 * n + 8 + 3 * ((n + 1) / 2)), "detailed wire width");
    std::vector<double> bristle;
    bristle.reserve(n);
for (std::size_t i = 0; i < n; ++i) bristle.push_back(row[column(names, std::format("front.ray.{:03}.bristle_m", i))]);
    const auto finite = [](double x) { return std::isfinite(x); };
    bt::require(std::ranges::any_of(bristle, finite) && !std::ranges::all_of(bristle, finite), "only detailed covered rays have finite bristles");
    Rig copy("detailed"); copy.tire->set_state(names, row); numeric_equal(copy.tire->state(), row, true);
    for (const auto &patch : Access::wheel(*rig.tire, 0).patch_states()) {
        bt::require(patch.has_detailed && patch.start >= 0 && patch.stop >= patch.start && std::cmp_less_equal(patch.stop, n), "valid detailed patch bounds");
        numeric_equal(std::span(bristle).subspan(static_cast<std::size_t>(patch.start), static_cast<std::size_t>(patch.stop - patch.start)), patch.bristle_deflection_m);
        const auto &rebuilt = patch_at(Access::wheel(*copy.tire, 0), patch.start, patch.stop);
        bt::require(rebuilt.has_detailed && rebuilt.detailed_kappa_prime == patch.detailed_kappa_prime, "detailed slip memory round trips");
        numeric_equal(rebuilt.bristle_deflection_m, patch.bristle_deflection_m);
    }
    Rig fast; (void)fast.advance(); const auto fast_row = fast.tire->state();
    for (std::size_t i = 0; i < 64; ++i)
        bt::require(std::isnan(fast_row[column(fast.tire->state_names(), std::format("front.ray.{:03}.bristle_m", i))]), "fast bristles remain NaN");
}
[[gnu::noinline]] void test_detailed_tier_leaves_model_timestep_untouched() {
    // test_pneumatic_tire_forces.py::test_detailed_tier_leaves_model_timestep_untouched
    Rig rig("detailed", .0007);
    bt::require(Access::config(*rig.tire).pneumatic->tier == "detailed", "detailed tier selected"); // NOLINT(bugprone-unchecked-optional-access) optional engagement is guaranteed by the fixture/request contract.
    bt::require(rig.engine.model->opt.timestep == .0007, "writer does not rewrite model timestep");
}
[[gnu::noinline]] void test_set_state_rejects_out_of_contract_cells() {
    // test_pneumatic_tire_forces.py::test_set_state_rejects_out_of_contract_cells
    Rig rig("detailed"); (void)rig.advance();
    const auto &names = rig.tire->state_names(); const auto row = rig.tire->state();
    std::vector<bool> covered(Access::wheel(*rig.tire, 0).n_rays(), false);
    for (const auto &patch : Access::wheel(*rig.tire, 0).patch_states())
        for (int ray = patch.start; ray < patch.stop; ++ray) {
            bt::require(ray >= 0 && static_cast<std::size_t>(ray) < covered.size(), "covered ray in ring");
            covered[static_cast<std::size_t>(ray)] = true;
        }
    const auto outside = std::ranges::find(covered, false);
    bt::require(outside != covered.end(), "fixture includes an uncovered ray");
    const auto corrupt = [&](std::string_view name, double value, std::string_view error) {
        auto bad = row; bad[column(names, name)] = value;
        rejects<std::runtime_error>([&] { rig.tire->set_state(names, bad); }, error);
    };
    corrupt(std::format("front.ray.{:03}.bristle_m", outside - covered.begin()), 0., "bristle cells are set outside every patch");
    for (const auto leaf : {"start", "stop"}) {
        const auto name = std::string("front.patch.00.") + leaf;
        corrupt(name, row[column(names, name)] + .5, "bounds must be integers");
    }
    corrupt("front.rim.x_m", 0., "must be NaN while inactive");
    numeric_equal(rig.tire->state(), row, true);
    Rig fast; (void)fast.advance(); auto bad = fast.tire->state();
    bad[column(fast.tire->state_names(), "front.ray.000.bristle_m")] = 0.;
    rejects<std::runtime_error>([&] { fast.tire->set_state(fast.tire->state_names(), bad); }, "must be NaN on the lumped tier");
}
[[gnu::noinline]] void test_rim_event_wire_round_trip() {
    // test_pneumatic_tire_forces.py::test_rim_event_wire_round_trip
    Rig rig("fast", .0005, .055); (void)rig.advance();
    const auto &names = rig.tire->state_names(); const auto row = rig.tire->state();
    bt::require(rig.tire->diagnostic_storage()[0].rim_strike_active && row[column(names, "front.rim.active")] == 1., "deep press creates active rim event");
    for (const auto leaf : {"x_m", "speed_mps", "peak_load_n", "peak_force_n", "energy_j"})
        bt::require(std::isfinite(row[column(names, std::string("front.rim.") + leaf)]), "active rim event finite");
    Rig copy; copy.tire->set_state(names, row); numeric_equal(copy.tire->state(), row, true);
    const auto &rim = Access::wheel(*copy.tire, 0).carcass_state();
    bt::require(rim.rim_strike_active() && rim.rim_event_x_m() == Access::wheel(*rig.tire, 0).carcass_state().rim_event_x_m(), "rim semantic state rebuilt");
}
[[gnu::noinline]] void test_probe_leaves_state_and_clock_untouched() {
    // test_pneumatic_tire_forces.py::test_probe_leaves_state_and_clock_untouched
    Rig rig; (void)rig.advance();
    const auto before = rig.tire->state(); const auto *identity = &Access::wheel(*rig.tire, 0);
    const auto clock = Access::clock(*rig.tire);
    const auto force = rig.tire->probe_qfrc(rig.engine.data.get(), rig.dt());
    bt::require(std::ranges::all_of(force, [](double x) { return std::isfinite(x); }), "probe force finite");
    numeric_equal(rig.tire->state(), before, true);
    bt::require(Access::clock(*rig.tire) == clock && &Access::wheel(*rig.tire, 0) == identity, "probe preserves clock and live identity");
    const auto snapshots = rig.tire->probe_snapshots();
    bt::require(snapshots[0].has_value() && snapshots[1].has_value(), "probe publishes both wheels");
    rejects([&] { (void)rig.advance(); }, "once per increasing timestamp");
}
[[gnu::noinline]] void test_interval_gate_and_restart_clock() {
    // test_pneumatic_tire_forces.py::test_interval_gate_and_restart_clock
    Rig rig; (void)rig.advance();
    rejects([&] { (void)rig.advance(); }, "once per increasing timestamp");
    rig.engine.data->time += rig.dt(); (void)rig.advance();
    rejects([&] { (void)rig.advance(); }, "once per increasing timestamp");
    rig.tire->restart_clock(); (void)rig.advance();
    bt::require(Access::clock(*rig.tire) == rig.engine.data->time, "restart allows same time once");
}
[[gnu::noinline]] void test_reset_clears_history_and_counters() {
    // test_pneumatic_tire_forces.py::test_reset_clears_history_and_counters
    Rig rig; (void)rig.advance();
    bt::require(rig.tire->snapshots_committed() && Access::wheel(*rig.tire, 0).carcass_state().initialized(), "pre-reset committed state");
    rig.tire->reset();
    bt::require(!rig.tire->snapshots_committed() && !rig.tire->snapshots()[0] && !rig.tire->diagnostics()[0] &&
        !rig.tire->snapshots()[1] && !rig.tire->diagnostics()[1] && !Access::clock(*rig.tire), "reset clears published state and clock");
    bt::require(rig.tire->elastic_energy_j() == 0. && rig.tire->brush_loss_step_j() == 0. && rig.tire->radial_dissipation_power_w() == 0., "reset clears all counters");
    for (std::size_t side = 0; side < sides.size(); ++side) unloaded_state(Access::wheel(*rig.tire, side));
    (void)rig.advance(); bt::require(rig.tire->diagnostic_storage()[0].normal_load_n > 0., "reset supports clean same-time retry");
}
[[gnu::noinline]] void test_stored_energy_matches_committed_counter() {
    // test_pneumatic_tire_forces.py::test_stored_energy_matches_committed_counter
    Rig rig("detailed", .0005, .004, rear_radius() - .004); rig.spin("rear", 2.);
    bt::require(rig.tire->stored_energy(rig.engine.data.get()) > 0., "loaded pose already stores energy");
    (void)rig.advance();
    bt::require(!rig.tire->snapshot_storage()[0].patches.empty() && !rig.tire->snapshot_storage()[1].patches.empty(), "both energy-test wheels loaded");
    bt::require(rig.tire->diagnostic_storage()[1].shear_energy_j > 0., "rear shear term is nonzero");
    bt::require(rig.tire->stored_energy(rig.engine.data.get()) == rig.tire->elastic_energy_j() && rig.tire->elastic_energy_j() > 0., "stored energy equals committed fold exactly");
    bt::require(rig.tire->brush_loss_step_j() >= 0., "nonnegative brush loss");
    rig.engine.data->time += rig.dt(); (void)rig.advance();
    bt::require(rig.tire->stored_energy(rig.engine.data.get()) == rig.tire->elastic_energy_j(), "second-step energy equality");
}
[[gnu::noinline]] void test_failure_atomicity_restores_front_state() {
    // test_pneumatic_tire_forces.py::test_failure_atomicity_restores_front_state
    Rig rig("fast", .0005, .004, rear_radius() - .004);
    Access::rear_count_control(*rig.tire, true);
    rejects([&] { (void)rig.advance(); }, "carcass state does not match the ray ring");
    unloaded_state(Access::wheel(*rig.tire, 0));
    bt::require(!Access::clock(*rig.tire) && !rig.tire->snapshots_committed(), "late rear rejection rolls back publication and clock");
    Access::rear_count_control(*rig.tire, false);
    const auto actual = rig.advance();
    bt::require(rig.tire->diagnostic_storage()[1].normal_load_n > 0., "valid rear retry carries load");
    Rig clean("fast", .0005, .004, rear_radius() - .004);
    numeric_equal(actual, clean.advance()); numeric_equal(rig.tire->state(), clean.tire->state(), true);
}
[[gnu::noinline]] void test_determinism_across_identical_rigs() {
    // test_pneumatic_tire_forces.py::test_determinism_across_identical_rigs
    Rig a, b;
    for (auto *rig : {&a, &b}) {
        for (int step = 0; step < 3; ++step) {
            const auto force = rig->advance();
            std::ranges::copy(force, model_access::mutable_buffer(rig->engine.data->qfrc_applied, rig->engine.model->nv).begin());
            engine::step(rig->engine.model.get(), rig->engine.data.get());
            engine::forward(rig->engine.model.get(), rig->engine.data.get());
        }
    }
    numeric_equal(a.tire->probe_qfrc(a.engine.data.get(), a.dt()), b.tire->probe_qfrc(b.engine.data.get(), b.dt()));
    numeric_equal(a.tire->state(), b.tire->state(), true);
}
} // namespace
// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() { return bt::run([] {
    test_constructor_rejects_wrong_backend_and_missing_surface();
    test_constructor_validates_contact_geom_contype_and_radius();
    test_qfrc_sanity_on_a_physical_model();
    test_snapshot_schema_and_signed_slip();
    test_state_wire_names_and_round_trip();
    test_detailed_tier_bristle_wire();
    test_detailed_tier_leaves_model_timestep_untouched();
    test_set_state_rejects_out_of_contract_cells();
    test_rim_event_wire_round_trip();
    test_probe_leaves_state_and_clock_untouched();
    test_interval_gate_and_restart_clock();
    test_reset_clears_history_and_counters();
    test_stored_energy_matches_committed_counter();
    test_failure_atomicity_restores_front_state();
    test_determinism_across_identical_rigs();
}); }
