// T27 authored regressions. No build or execution was performed while
// implementing this wave; these are future acceptance cases, not pass evidence.
#include "support/fixture_test.hpp"
#include "diag.hpp"
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

NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void native_topologies() {
    const auto available = drivetrain::implemented_capabilities();
    bt::require(available.motor_clutch && available.reflected_rotor,
        "native availability includes existing clutch and rotor branches");
    struct Variant { std::string_view family; bool clutch; double rotor; };
    constexpr std::array variants{
        Variant{.family="ideal_mid_drive", .clutch=false, .rotor=0.},
        Variant{.family="ideal_mid_drive", .clutch=true, .rotor=0.},
        Variant{.family="ideal_mid_drive", .clutch=false, .rotor=.02},
        Variant{.family="geometric_ideal_mid_drive", .clutch=false, .rotor=0.},
        Variant{.family="geometric_ideal_mid_drive", .clutch=true, .rotor=0.},
        Variant{.family="geometric_ideal_mid_drive", .clutch=false, .rotor=.02}};
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
        bt::require(std::views::counted(model->actuator_trnid, 2 * model->nu)[2U*static_cast<std::size_t>(actuator)] == target,
            "motor actuator targets the selected crank, shaft or rotor joint");

        drivetrain::OwnedData const data(engine::make_data(model.get()));
        engine::forward(model.get(), data.get());
        auto writer = std::make_unique<drivetrain::DrivetrainWriter>(model.get(), data.get(), *config.drive);
        writer->reset();
        const auto initial = std::make_unique<drivetrain::DriveSnapshot>(writer->state());
        bt::require(initial->ideal_hub.has_value(),
            "reset state carries the ideal freehub topology");
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
            for (double const value : component.values) bt::require(std::isfinite(value), "finite staged force");
        }
        auto malformed = std::make_unique<drivetrain::DriveSnapshot>(*initial);
        // A shift count without its shift clock and direction is rejected
        // by the snapshot validation that runs before any state moves.
        malformed->shifting.shift_count = 1;
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
    return bt::run([] { native_topologies(); });
}
NATIVE_DIAG_POP
