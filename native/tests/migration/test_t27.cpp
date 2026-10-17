// T27 authored regressions. No build or execution was performed while
// implementing this wave; these are future acceptance cases, not pass evidence.
#include "support/migration_test.hpp"
#include "configuration/load.hpp"
#include "drivetrain/capabilities.hpp"
#include "drivetrain/chain.hpp"
#include "drivetrain/state_codec.hpp"
#include "drivetrain/transmission.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include "model/builder.hpp"
#include "model/mapping.hpp"
#include "runtime/project_config.hpp"
#include "terrain/trackfile.hpp"
#include "writer_config.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <variant>

namespace {

void chain_geometry_and_virtual_work() {
    constexpr std::array<double, 4> q{0., 0., .4, .1};
    const auto evaluate = [](std::span<const double> value) {
        return drivetrain::chain_extension({value[0], 0.}, {.5, value[1]},
            .05, .05, value[2], value[3], .5);
    };
    bt::require(std::abs(evaluate(q) - .015) < 1e-15,
        "equal sprockets have tangent length plus relative angular extension");
    const auto jacobian = drivetrain::chain_jacobian(q, evaluate);
    constexpr std::array<double, 4> expected{-1., 0., .05, -.05};
    for (std::size_t i = 0; i < expected.size(); ++i)
        bt::require(std::abs(jacobian[i] - expected[i]) < 1e-9,
            "independent central difference matches analytic chain derivative");

    const auto linear = drivetrain::linearized_upper_bound(evaluate(q), jacobian, q, .2);
    bt::require(linear.first == jacobian, "linearized coefficients own the complete Jacobian");
    bt::require(std::abs(drivetrain::dot(linear.first, q) - linear.second -
                         (evaluate(q) - .2)) < 1e-15,
        "linearized tendon preserves the source gap at the expansion point");
    const auto reaction = drivetrain::constraint_reaction(80., jacobian);
    constexpr std::array<double, 4> velocity{.1, .3, 4., 1.};
    bt::require(std::abs(drivetrain::dot(reaction, velocity) +
                         80. * drivetrain::dot(jacobian, velocity)) < 1e-13,
        "chain generalized force supplies exactly minus tension times extension power");
    const auto slack = drivetrain::chain_tension(-.01, 3., 1000., 20.);
    bt::require(slack.first == 0. && slack.second == 0., "slack chain has no push or stored energy");
    const auto stretched = drivetrain::chain_tension(.02, -.1, 1000., 20.);
    bt::require(stretched.first == 18. && stretched.second == .2,
        "chain damping changes tensile force and not elastic stored energy");
    bt::invalid([&] { (void)drivetrain::linearized_upper_bound(0., jacobian, {}, 0.); });
    bt::invalid([&] { (void)drivetrain::constraint_reaction(-1., jacobian); });
    bt::invalid([&] { (void)drivetrain::chain_jacobian(q, evaluate, 0.); });
    bt::invalid([&] { (void)drivetrain::chain_jacobian(q, {}); });
    bt::invalid([&] {
        (void)drivetrain::chain_extension({}, {.5, 0.}, .05, .05,
            std::numeric_limits<double>::quiet_NaN(), 0., 0.);
    });
}

void native_topologies() {
    const auto available = drivetrain::implemented_capabilities();
    bt::require(available.elastic_chain && available.motor_clutch && available.reflected_rotor,
        "native availability includes existing elastic, clutch and rotor branches");
    struct Variant { std::string_view family; bool clutch; double rotor; };
    constexpr std::array variants{
        Variant{"elastic_chain", false, 0.},
        Variant{"ideal_mid_drive", false, 0.},
        Variant{"ideal_mid_drive", true, 0.},
        Variant{"ideal_mid_drive", false, .02},
        Variant{"geometric_ideal_mid_drive", false, 0.},
        Variant{"geometric_ideal_mid_drive", true, 0.},
        Variant{"geometric_ideal_mid_drive", false, .02}};
    const auto track = bike::terrain::load_track(bt::fixture("tracks/flat.toml"));
    for (const auto& variant : variants) {
        auto request = std::make_unique<bike::model::BuildRequest>();
        request->mode = "ride";
        request->rider = bike::configuration::RiderSpecs{};
        request->rider->variant = "articulated_planar";
        request->physics = bike::configuration::load_physics(bt::fixture("config/viewer_physics_welded.toml"));
        auto& source = request->physics->drive;
        source.transmission_model = variant.family;
        source.motor_clutch = variant.clutch;
        source.rotor_inertia_kgm2 = variant.rotor;
        source.shifting.enabled = false;
        runtime::validate_native_supported(*request->physics, *request->rider);
        auto model = bike::model::compile_mjcf(bike::model::generate_mjcf(*request));
        const auto config = nativecfg::decode(runtime::project_writer_config(*request, *model, track));
        bt::require(config.drive.has_value(), "resolved source config enables the physical drive writer");
        const auto topology = drivetrain::motor_topology(*config.drive);
        const int actuator = bike::model::required_id(*model, mjOBJ_ACTUATOR, "mid_drive");
        const int target = bike::model::required_id(*model, mjOBJ_JOINT, topology.actuator_joint);
        bt::require(std::span<const int>(model->actuator_trnid,
            static_cast<std::size_t>(2 * model->nu))[static_cast<std::size_t>(2 * actuator)] == target,
            "motor actuator targets the selected crank, shaft or rotor joint");

        drivetrain::OwnedData data(engine::make_data(model.get()));
        engine::forward(model.get(), data.get());
        auto writer = std::make_unique<drivetrain::DrivetrainWriter>(model.get(), data.get(), *config.drive);
        writer->reset();
        const auto initial = std::make_unique<drivetrain::DriveSnapshot>(writer->state());
        const bool elastic = variant.family == "elastic_chain";
        bt::require(initial->hub.has_value() == elastic && initial->ideal_hub.has_value() != elastic,
            "reset state carries exactly its selected freehub topology");
        bt::require(initial->clutch.has_value() == variant.clutch &&
            initial->freewheel.has_value() == (variant.rotor > 0.),
            "auxiliary clutch and rotor state are neither dropped nor invented");
        const auto before = bike::io::canonical_json(drivetrain::encode_drive_snapshot(*initial));

        drivetrain::TickInputs tick;
        tick.dt = model->opt.timestep;
        tick.active = false;
        tick.advance = false;
        const auto candidate = writer->stage_components(tick);
        bt::require(before == bike::io::canonical_json(drivetrain::encode_drive_snapshot(writer->state())),
            "staged topology probe does not publish policy, energy or transmission state");
        for (const auto& component : candidate.components) {
            bt::require(component.values.size() == static_cast<std::size_t>(model->nv),
                "force component width matches the owning model");
            for (double value : component.values) bt::require(std::isfinite(value), "finite staged force");
        }
        auto malformed = std::make_unique<drivetrain::DriveSnapshot>(*initial);
        malformed->battery.energy_j = -1.;
        bt::invalid([&] { writer->restore(*malformed); });
        bt::require(before == bike::io::canonical_json(drivetrain::encode_drive_snapshot(writer->state())),
            "failed state restore leaves the writer unchanged");
        writer->restore(*initial);
        bt::require(before == bike::io::canonical_json(drivetrain::encode_drive_snapshot(writer->state())),
            "complete drive state roundtrips across each transmission family");

        auto invalid = *config.drive;
        invalid.motor_clutch = true;
        invalid.rotor_inertia_kgm2 = .02;
        bt::invalid([&] { (void)drivetrain::motor_topology(invalid); });
        invalid = *config.drive;
        invalid.transmission_model = "elastic_chain";
        invalid.motor_clutch = true;
        bt::invalid([&] { (void)drivetrain::motor_topology(invalid); });
    }
}
} // namespace

int main() {
    return bt::run([] { chain_geometry_and_virtual_work(); native_topologies(); });
}
