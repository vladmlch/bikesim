// W19 source authoring. V03/V05 execute this matrix only after the W20/W21/
// W27 producers are linked. Missing captures/producers never become a skip.
#include "support/migration_test.hpp"
#include "support/equilibrium_cases.hpp"
#include "support/model_request.hpp"
#include "support/rider_fixture.hpp"
#include "support/spatial_fixture.hpp"
#include "support/startup_fixture.hpp"
#include "support/terrain_fixture.hpp"
#include "configuration/load.hpp"
#include "configuration/model_config.hpp"
#include "diag.hpp"
#include "drivetrain/state_codec.hpp"
#include "engine_call.hpp"
#include "initialization/capture.hpp"
#include "io/json.hpp"
#include "model_access.hpp"
#include "pneumatic3d/carcass.hpp"
#include "pneumatic3d/state.hpp"
#include "research/evidence.hpp"
#include "runtime/bootstrap_io.hpp"
#include "runtime/capabilities.hpp"
#include "runtime/runtime.hpp"
#include "runtime/setup_values.hpp"
#include "runtime/spatial_checkpoint_internal.hpp"
#include "runtime/spatial_tire_factory.hpp"
#include "spatial/frames.hpp"
#include "spatial/topology.hpp"
#include "validation/runtime_access.hpp"
#include "writers/pneumatic_3d_tire.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numbers>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {
namespace sp=bike::spatial;
namespace tyre=bike::pneumatic3d;
namespace cfg=bike::configuration;
namespace io=bike::io;
using namespace runtime::setupvalue;

template<class F> void invalid_path(F&& operation,std::string_view path) {
    bt::invalid([&] {
        try { std::forward<F>(operation)(); }
        catch(const std::invalid_argument& error) {
            bt::require(std::string_view(error.what()).find(path)!=std::string_view::npos,
                "rejection must name "+std::string(path)+": "+error.what());
            throw;
        }
    });
}
void append(std::vector<std::byte>& target,std::span<const std::byte> bytes) {
    // Length prefixes make concatenated variable-width owners unambiguous.
    const auto width=static_cast<std::uint64_t>(bytes.size());
    const auto prefix=std::as_bytes(std::span(&width,1));
    target.insert(target.end(),prefix.begin(),prefix.end());
    target.insert(target.end(),bytes.begin(),bytes.end());
}
void append(std::vector<std::byte>& target,const Value& value) {
    const auto json=io::canonical_json(value);
    append(target,std::as_bytes(std::span(json.data(),json.size())));
}
std::vector<std::byte> model_state(const mjModel& model,const mjData& data) {
    auto image=bike::initialization::capture_model_image(model);
    const auto count=engine::state_size(&model,mjSTATE_INTEGRATION);
    bt::require(count>=0,"engine state has a nonnegative width");
    std::vector<double> integration(static_cast<std::size_t>(count));
    engine::get_state(&model,&data,integration.data(),mjSTATE_INTEGRATION);
    append(image.mjb,std::as_bytes(std::span(integration)));
    return std::move(image.mjb);
}

// This is a real published runtime, deliberately kept at its captured t=0.
// The fixture bytes are pre-existing planar evidence; no synthetic counter
// stands in for a publisher and no spatial capability is inferred from it.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
struct Witness {
    std::unique_ptr<runtime::RuntimeBootstrap> initial;
    std::unique_ptr<runtime::NativeRideRuntime> ride;
    Witness() : initial(std::make_unique<runtime::RuntimeBootstrap>(
        runtime::read_bootstrap(bt::fixture("bootstrap/welded")))),
        ride(std::make_unique<runtime::NativeRideRuntime>(*initial)) {}
    std::vector<std::byte> bytes() {
        std::vector<std::byte> bytes;
        bike::validation::RuntimeRigAccess::inspect(*ride,[&](const Stepper& stepper,const runtime::PhysicalStep& physical) {
            // Reuse production capture and codecs for every causal owner;
            // a second hand-maintained subset of controller fields is unsafe.
            const auto captured=std::make_unique<runtime::RuntimeBootstrap>(
                bike::initialization::capture_initialized(stepper,physical,initial->config));
            bytes=model_state(*stepper.model(),*stepper.data());
            const auto& state=captured->state;
            append(bytes,Value{Object{
                {"drive",drivetrain::encode_drive_snapshot(state.drive)},
                {"rider_contacts",state.rider_contacts},{"rider_controller",state.rider_controller},
                {"rider_intent",state.rider_intent},{"signals",state.signals},{"runtime",state.runtime}}});
            Array names;for(const auto& name:state.tire.names)names.push_back(text(name));
            append(bytes,Value{std::move(names)});
            append(bytes,std::as_bytes(std::span(state.tire.row)));
        });
        append(bytes,bike::research::wire_value(runtime::Wire{ride->accounting_state()}));
        const auto snapshot=std::make_unique<runtime::RuntimeSnapshot>(ride->snapshot());
        append(bytes,Value{Object{{"generation",integer(snapshot->generation)},{"step",integer(snapshot->step)},
            {"time",real(snapshot->time_s)},{"model",text(snapshot->model_sha256)},
            {"outcome",snapshot->outcome?text(*snapshot->outcome):Value{}},
            {"view",bike::research::wire_value(runtime::Wire{snapshot->view})},
            {"model_status",bike::research::wire_value(runtime::Wire{snapshot->model_status})}}});
        append(bytes,snapshot->tire_state_bytes);
        const auto publications=ride->prepare_samples();
        append(bytes,integer(publications.generation));
        for(const auto& sample:publications.samples)
            append(bytes,bike::research::wire_value(runtime::Wire{sample->as_wire()}));
        return bytes;
    }
    std::pair<std::size_t,std::size_t> publications() const {
        return {ride->prepare_samples().samples.size(),ride->recorded_intervals().size()};
    }
};
NATIVE_DIAG_POP

template<class F> void reject(Witness& witness,const spatial_tests::RigModel& rig,
                              std::string_view path,F&& operation) {
    const auto runtime_before=witness.bytes();
    const auto model_before=spatial_tests::complete_state_bytes(rig);
    const auto publications_before=witness.publications();
    invalid_path(std::forward<F>(operation),path);
    bt::require(model_before==spatial_tests::complete_state_bytes(rig),"rejected input preserved complete engine state");
    bt::require(runtime_before==witness.bytes(),"rejected input preserved complete causal runtime state");
    bt::require(publications_before==witness.publications(),"rejected input preserved actual publication inventory");
}

sp::Config free_config() {
    sp::Config config;config.mode=sp::Mode::spatial3d;config.root=sp::RootKind::free6;
    config.steer.controller=sp::SteerController::off;return config;
}
runtime::SpatialBootstrapState causal_state(const spatial_tests::RigModel& rig,
    const bike::terrain::Terrain3dSource& terrain) {
    runtime::SpatialBootstrapState state;state.root_kind=rig.topology.root.kind;
    state.topology_fingerprint=rig.topology.fingerprint_hex;state.terrain_sha256=terrain.fingerprint_hex();
    state.bootstrap_phase=state.hold.phase;return state;
}
runtime::RuntimeConfig cold_config(const sp::Config& spatial,const runtime::SpatialBootstrapState& state,
    std::shared_ptr<const bike::terrain::Terrain3dSource> terrain) {
    runtime::RuntimeConfig config;config.schema=2;config.spatial=spatial;config.terrain3d=std::move(terrain);
    // Only the cold version/identity validator consumes these empty producer
    // sections. They are never presented as a constructible tyre writer.
    config.writer_config=Value{Object{{"schema",integer(3)}, {"spatial",Value{Object{
        {"spatial_contract",integer(1)},{"root_kind",text(sp::to_string(spatial.root))},
        {"topology_fingerprint",text(state.topology_fingerprint)},{"terrain_sha256",text(state.terrain_sha256)},
        {"numeric_profile",text("binary64")}}}, {"drive",Value{Object{}}},
        {"suspension",Value{Object{}}},{"resistance",Value{Object{}}},
        {"tire",Value{Object{{"backend",text("pneumatic_3d")},{"pneumatic_3d",Value{Object{}}}}}}};
    return config;
}

void topology_matrix() {
    Witness witness;
    const auto config=free_config();
    auto rig=spatial_tests::free_rig(config,false);
    auto types=model_access::mutable_buffer(rig.model->jnt_type,rig.model->njnt);
    const auto root=static_cast<std::size_t>(rig.topology.root.free_joint_id);
    types[root]=mjJNT_BALL;
    reject(witness,rig,"topology.root_free",[&]{(void)sp::bind_topology(*rig.model,config);});
    types[root]=mjJNT_FREE;
    auto names=model_access::mutable_buffer(rig.model->name_jntadr,rig.model->njnt);
    auto characters=model_access::mutable_buffer(rig.model->names,rig.model->nnames);
    const auto root_name=static_cast<std::size_t>(names[root]);
    const char root_initial=characters[root_name];characters[root_name]='x';
    reject(witness,rig,"topology.root_free",[&]{(void)sp::bind_topology(*rig.model,config);});
    characters[root_name]=root_initial;
    const auto front=static_cast<std::size_t>(rig.topology.wheels[0].spin.joint_id);
    const auto rear=static_cast<std::size_t>(rig.topology.wheels[1].spin.joint_id);
    const int rear_name=names[rear];names[rear]=names[front];
    reject(witness,rig,"topology.front_wheel_spin",[&]{(void)sp::bind_topology(*rig.model,config);});
    names[rear]=rear_name;
    auto targets=model_access::mutable_buffer(rig.model->actuator_trnid,2*rig.model->nu);
    const auto target=2*static_cast<std::size_t>(rig.topology.steer.actuator_id);
    const int original=targets[target];targets[target]=rig.topology.wheels[1].spin.joint_id;
    reject(witness,rig,"topology.steer_torque",[&]{(void)sp::bind_topology(*rig.model,config);});
    targets[target]=original;
    auto constrained=config;constrained.root=sp::RootKind::constrained4;
    auto scalar=spatial_tests::constrained_rig(constrained);
    auto axes=model_access::mutable_buffer(scalar.model->jnt_axis,3*scalar.model->njnt);
    const auto axis=3*static_cast<std::size_t>(scalar.topology.root.scalar[1].joint_id);
    axes[axis]=1.;axes[axis+1]=0.;
    reject(witness,scalar,"topology.root_y",[&]{(void)sp::bind_topology(*scalar.model,constrained);});
}

spatial_tests::RigModel reordered(bool reverse) {
    constexpr std::string_view front=R"(<body name="steer" pos=".6 0 0"><joint name="steer_joint" axis="0 0 1"/><geom size=".05" mass="1"/><body name="front_wheel"><joint name="front_wheel_spin" axis="0 1 0"/><geom size=".3" mass="2"/></body></body>)";
    constexpr std::string_view rear=R"(<body name="rear_hub" pos="-.6 0 0"><geom size=".05" mass="1"/><body name="rear_wheel"><joint name="rear_wheel_spin" axis="0 1 0"/><geom size=".3" mass="2"/></body></body>)";
    std::string xml=R"(<mujoco><default><geom contype="0" conaffinity="0"/></default><worldbody><body name="frame"><freejoint name="root_free"/><geom size=".1" mass="10"/>)";
    xml+=reverse?rear:front;xml+=reverse?front:rear;
    xml+=R"(</body></worldbody><actuator><motor name="steer_torque" joint="steer_joint"/></actuator></mujoco>)";
    spatial_tests::RigModel rig;rig.model=bike::model::compile_mjcf(xml).release();rig.data=engine::make_data(rig.model);
    bt::require(rig.data!=nullptr,"reordered fixture owns engine state");engine::forward(rig.model,rig.data);
    rig.topology=sp::bind_topology(*rig.model,free_config());return rig;
}

void identity_and_entrypoints() {
    Witness witness;auto rig=reordered(false);auto other=reordered(true);
    bt::require(rig.model->nq==other.model->nq && rig.model->nv==other.model->nv &&
        rig.model->nbody==other.model->nbody && rig.model->njnt==other.model->njnt &&
        rig.model->nu==other.model->nu && rig.model->ngeom==other.model->ngeom,
        "reordered models preserve dimensions while changing identity");
    bt::require(rig.topology.fingerprint_hex!=other.topology.fingerprint_hex,"joint/body order changes topology fingerprint");
    auto terrain=spatial_tests::flat_model_ground();auto state=causal_state(rig,*terrain);
    auto config=cold_config(free_config(),state,terrain);
    runtime::validate_schema_family(config,2);runtime::validate_spatial_identity(config,state,rig.topology);
    reject(witness,rig,"state.spatial.topology_fingerprint",[&]{runtime::validate_spatial_identity(config,state,other.topology);});
    auto stale=state;stale.terrain_sha256=std::string(64,'0');
    reject(witness,rig,"state.spatial.terrain_sha256",[&]{runtime::validate_spatial_identity(config,stale,rig.topology);});
    auto wrong_family=config;wrong_family.schema=1;
    reject(witness,rig,"bootstrap.schema_family",[&]{runtime::validate_schema_family(wrong_family,2);});

    for(bool constrained:{false,true}) {
        const auto path=constrained?"spatial_3d.steer.controller":"tires.backend";
        auto request=spatial_tests::model_request(constrained?sp::RootKind::constrained4:sp::RootKind::free6);
        if(constrained) request->spatial->steer.controller=sp::SteerController::roll_heading;
        else request->physics->tires.backend="pneumatic";
        reject(witness,rig,path,[&]{(void)bike::model::generate_mjcf(*request);});
        const auto name=constrained?"config/neg_constrained_controller.toml":"config/neg_3d_planar_tyre.toml";
        reject(witness,rig,path,[&]{
            // NOLINTNEXTLINE(modernize-make-unique) direct prvalue construction avoids a large stack copy
            const std::unique_ptr<cfg::SimulationPhysicsConfig> parsed(new cfg::SimulationPhysicsConfig(
                cfg::load_physics(spatial_tests::fixture(name))));
            (void)parsed;
        });
        auto saved=runtime::resolve_bootstrap_document(
            io::parse_json(bt::text(bt::fixture("bootstrap/welded/config.json"))),bt::fixture("bootstrap/welded"));
        auto saved_writer=config.writer_config;
        set(saved_writer,"drive",field(field(saved,"writer_config"),"drive"));
        set(saved,"runtime_schema",integer(2));set(saved,"writer_config",std::move(saved_writer));
        auto spatial_wire=sp::spatial_config_to_wire(free_config());
        if(constrained) {
            set(spatial_wire,"root",text("constrained"));
            auto steer=field(spatial_wire,"steer");set(steer,"controller",text("roll_heading"));
            set(spatial_wire,"steer",std::move(steer));set(saved,"spatial",std::move(spatial_wire));
            reject(witness,rig,"config.spatial.steer.controller",[&]{(void)runtime::decode_runtime_config(saved);});
        } else {
            set(saved,"spatial",std::move(spatial_wire));
            auto writer_doc=field(saved,"writer_config");auto tire_doc=field(writer_doc,"tire");
            set(tire_doc,"backend",text("pneumatic"));set(writer_doc,"tire",std::move(tire_doc));
            set(saved,"writer_config",std::move(writer_doc));
            reject(witness,rig,"config.tire.backend",[&]{(void)runtime::decode_runtime_config(saved);});
        }
    }
    // The replay/import construction seam receives actual owned model bytes.
    // Bad version-family input rejects before it can construct an owner.
    auto saved_document=runtime::resolve_bootstrap_document(
        io::parse_json(bt::text(bt::fixture("bootstrap/welded/state.json"))),bt::fixture("bootstrap/welded"));
    set(saved_document,"schema",integer(2));
    auto replay_config=config;replay_config.schema=1;
    reject(witness,rig,"bootstrap.schema_family",[&]{
        // NOLINTNEXTLINE(modernize-make-unique) direct result construction avoids a large stack temporary
        const std::unique_ptr<runtime::RuntimeBootstrap> staged(new runtime::RuntimeBootstrap(
            runtime::compose_bootstrap(replay_config,witness.initial->model,saved_document)));
        (void)staged;
    });
}

void producer_declaration_mismatch() {
    for(const auto tier:{sp::TyreTier::fast,sp::TyreTier::detailed}) {
        auto arena=std::make_unique<spatial_tests::EquilibriumArena>(sp::RootKind::constrained4,tier,false);
        bt::require(arena->config.spatial.has_value() && arena->stepper->has_spatial_tire(),
            "production arena installs the declared spatial tyre owner");
        const auto image=bike::initialization::capture_model_image(*arena->stepper->model());
        auto cold=std::make_unique<Stepper>(image.mjb);
        const auto reject_declarations=[&](Stepper& target) {
            const bool installed=target.has_spatial_tire();
            const auto engine_before=model_state(*target.model(),*target.data());
            const auto material_before=installed?target.spatial_tire_writer().state_bytes():std::vector<std::byte>{};
            const auto rejected=[&](const runtime::RuntimeConfig& wrong,std::string_view path) {
                // Only the runtime declaration changes: the validated writer
                // payload, terrain and coefficient digest remain exact.
                invalid_path([&]{runtime::install_spatial_tire(target,wrong);},path);
                bt::require(target.has_spatial_tire()==installed,
                    "declaration mismatch does not install or replace a contact producer");
                bt::require(engine_before==model_state(*target.model(),*target.data()),
                    "declaration rejection preserves the complete engine model and state");
                if(installed)bt::require(material_before==target.spatial_tire_writer().state_bytes(),
                    "declaration rejection preserves the installed producer identity and history");
            };
            auto wrong=std::make_unique<runtime::RuntimeConfig>(arena->config);
            wrong->spatial->tyre.tier=tier==sp::TyreTier::fast?sp::TyreTier::detailed:sp::TyreTier::fast;
            rejected(*wrong,"config.tire.pneumatic_3d.tier");
            *wrong=arena->config;
            wrong->spatial->tyre.lateral_coefficients_id="mtb-magic-mary-29x2.4-front";
            bt::require(wrong->spatial->tyre.lateral_coefficients_id!=arena->config.spatial->tyre.lateral_coefficients_id,
                "profile negative selects a different registered declaration");
            (void)cfg::spatial_tyre_coefficient_profile(wrong->spatial->tyre.lateral_coefficients_id);
            rejected(*wrong,"config.tire.pneumatic_3d.coefficient_profile");
            runtime::install_spatial_tire(target,arena->config);
            bt::require(target.has_spatial_tire() && target.spatial_tire_writer().state().identity.tier==tier,
                "matching declaration succeeds after rejected candidates");
            if(installed)bt::require(material_before==target.spatial_tire_writer().state_bytes(),
                "reinstalling the exact existing producer is observational");
        };
        reject_declarations(*cold);
        // The same real cold owner now exercises the existing-producer gate;
        // no initialization pass mutates its model between these calls.
        reject_declarations(*cold);
    }
}

tyre::TyreState3d zero_material(const spatial_tests::RigModel& rig,
                              const bike::terrain::Terrain3dSource& terrain) {
    auto request=spatial_tests::model_request(rig.topology.root.kind);
    request->physics->spatial=request->spatial;
    cfg::post_init(*request->physics);
    runtime::resolve_spatial_runtime_profiles(*request->physics);
    const auto wire=runtime::project_spatial_tire_config(*request->physics);
    auto spec=runtime::decode_spatial_tire_config(wire);
    tyre::TyreStateIdentity3d identity;
    identity.tier=spec.tier;identity.dims=tyre::tier_dims(spec.tier);
    identity.patch_capacity=identity.dims.n_theta_fan*identity.dims.n_phi;
    identity.topology_fingerprint_hex=rig.topology.fingerprint_hex;
    identity.terrain_sha256_hex=terrain.fingerprint_hex();
    identity.config_fingerprint_hex=sp::spatial_config_fingerprint_hex(*request->physics->spatial);
    identity.calibration_sha256_hex=request->physics->spatial->tyre.lateral_coefficients_sha256;
    // The production owner supplies every dimension, unused-slot sentinel and
    // canonical zero; its digest identifies resolved provisional coefficients.
    auto model=std::make_unique<tyre::Pneumatic3dModel>(std::move(spec),terrain.query_view(),std::move(identity));
    return model->committed_state();
}
Value dimensions(const mjModel& model) {
    return Value{Object{{"nq",integer(model.nq)},{"nv",integer(model.nv)},{"nu",integer(model.nu)},
        {"na",integer(model.na)},{"nbody",integer(model.nbody)},{"njnt",integer(model.njnt)},
        {"ntendon",integer(model.ntendon)},{"nwrap",integer(model.nwrap)},{"nsite",integer(model.nsite)},
        {"ngeom",integer(model.ngeom)},{"neq",integer(model.neq)},{"nsensor",integer(model.nsensor)},
        {"nsensordata",integer(model.nsensordata)}}};
}
void bootstrap_quaternion() {
    Witness witness;auto rig=spatial_tests::mixed_joint_rig();
    auto terrain=std::make_shared<const bike::terrain::Terrain3dSource>(spatial_tests::plane_grid(0.,0.,0.,33,17,.25,.25));
    auto spatial=causal_state(rig,*terrain);const auto material=zero_material(rig,*terrain);
    spatial.calibration_sha256=material.identity.calibration_sha256_hex;
    const auto bytes=tyre::encode_tyre_state(material);
    const auto restored=tyre::decode_tyre_state(bytes,material.identity);
    bt::require(restored.has_value(),"real typed empty material state is a valid W27 codec control");
    auto doc=runtime::resolve_bootstrap_document(io::parse_json(bt::text(bt::fixture("bootstrap/welded/state.json"))),
        bt::fixture("bootstrap/welded"));
    set(doc,"schema",integer(2));set(doc,"model_digest",text(rig.topology.model_sha256_hex));
    set(doc,"model_dims",dimensions(*rig.model));
    set(doc,"model_mutable",Value{Object{{"dof_frictionloss",Value{Object{}}},{"site_pos",Value{Object{}}}}});
    set(doc,"spatial",runtime::encode_spatial_bootstrap(spatial));
    set(doc,"tire",Value{Object{{"pneumatic3d_state",integer(1)},
        {"identity",tyre::tyre_identity_to_wire(material.identity)},{"encoding",text("base64")},
        {"payload",text(tyre::base64_canonical(bytes))}}});
    const auto count=engine::state_size(rig.model,mjSTATE_INTEGRATION);std::vector<double> integration(static_cast<std::size_t>(count));
    engine::get_state(rig.model,rig.data,integration.data(),mjSTATE_INTEGRATION);set(doc,"integration_state",reals(integration));
    // The remaining causal documents are real recorded inputs to their cold
    // decoders. This control is not a runnable mixed-rig physical bootstrap.
    // NOLINTNEXTLINE(modernize-make-unique) direct result construction avoids a large stack temporary
    const std::unique_ptr<runtime::BootstrapState> control(new runtime::BootstrapState(
        runtime::decode_bootstrap_state(doc,*rig.model)));
    (void)control;
    const auto joint_types=model_access::readonly_buffer(rig.model->jnt_type,rig.model->njnt);
    const auto addresses=model_access::readonly_buffer(rig.model->jnt_qposadr,rig.model->njnt);
    for(std::size_t joint=0;joint<joint_types.size();++joint) {
        if(joint_types[joint]!=mjJNT_FREE && joint_types[joint]!=mjJNT_BALL)continue;
        const auto offset=static_cast<std::size_t>(addresses[joint]+(joint_types[joint]==mjJNT_FREE?3:0));
        auto q=model_access::mutable_buffer(rig.data->qpos,rig.model->nq);const double original=q[offset];q[offset]=2.;
        engine::get_state(rig.model,rig.data,integration.data(),mjSTATE_INTEGRATION);q[offset]=original;
        auto invalid=doc;set(invalid,"integration_state",reals(integration));
        reject(witness,rig,"state.integration_state.qpos",[&]{
            // NOLINTNEXTLINE(modernize-make-unique) direct result construction avoids a large stack temporary
            const std::unique_ptr<runtime::BootstrapState> staged(new runtime::BootstrapState(
                runtime::decode_bootstrap_state(invalid,*rig.model)));
            (void)staged;
        });
    }
}

void shoulder_layout() {
    spatial_tests::RiderRig rider;
    const auto before=model_state(*rider.rig.model,*rider.rig.data);
    const auto original=rider.controller->state(rider.rig.data.get());
    const auto state_bytes=io::canonical_json(::rider::encode_spindle_state(original));
    const auto unchanged=[&] {
        bt::require(before==model_state(*rider.rig.model,*rider.rig.data) &&
            state_bytes==io::canonical_json(::rider::encode_spindle_state(rider.controller->state(rider.rig.data.get()))),
            "each incompatible shoulder state preserves the actual target controller/model/data");
    };
    const auto rejected=[&](const ::rider::SpindleController::State& incompatible) {
        invalid_path([&]{rider.controller->restore(incompatible);},"rider_controller.rider.spatial");
        unchanged();
    };
    auto incompatible=original;bt::require(incompatible.spatial.has_value(),"spatial controller owns shoulder lanes");
    incompatible.spatial->arm_branch*=-1;
    rejected(incompatible);
    incompatible=original;incompatible.spatial->shoulder_profile_sha256=std::string(64,'0');
    rejected(incompatible);
    incompatible=original;incompatible.spatial.reset();
    rejected(incompatible);
    rider.controller->restore(original);
    unchanged();
}

std::string publication_bytes(runtime::NativeRideRuntime& ride) {
    const auto pending=ride.prepare_samples();
    Array pending_rows,retained_rows;
    for(const auto& sample:pending.samples)
        pending_rows.push_back(bike::research::wire_value(runtime::Wire{sample->as_wire()}));
    for(const auto& sample:ride.recorded_intervals())
        retained_rows.push_back(bike::research::wire_value(runtime::Wire{sample->as_wire()}));
    return io::recording_json(Value{Object{{"generation",integer(pending.generation)},
        {"pending",Value{std::move(pending_rows)}},{"retained",Value{std::move(retained_rows)}},
        {"accounting",bike::research::wire_value(runtime::Wire{ride.accounting_state()})}}});
}

void public_probe_isolation() {
    auto request=spatial_tests::make_startup_request(sp::RootKind::free6,sp::TyreTier::fast,false,.5);
    auto fixture=std::make_unique<spatial_tests::OwnedStartup>(std::move(*request));
    auto reference=std::make_unique<runtime::NativeRideRuntime>(*fixture->bootstrap);
    runtime::RideControl drive;drive.human_torque_nm=1.;
    runtime::RideControl probe_control;probe_control.human_torque_nm=3.;
    runtime::RideControl invalid;invalid.motor_torque_nm=-1.;
    const auto probe=[&](runtime::NativeRideRuntime& ride) {
        const auto result=ride.probe_step_inputs(probe_control,.6,.4);
        bt::require(!result.components.empty() && !result.ctrl.empty() &&
            std::ranges::all_of(result.ctrl,[](double value){return std::isfinite(value);}),
            "free-root public probe returns an owned finite staged actuator row");
    };
    auto& ride=*fixture->runtime;
    (void)ride.advance(33,drive,.2,.1,{});
    (void)reference->advance(33,drive,.2,.1,{});
    const auto boundary=ride.snapshot();
    bt::require(boundary.step==33 && boundary.spatial &&
        boundary.spatial->steer.controller==sp::SteerController::roll_heading &&
        boundary.spatial->steer.next_tick>0 && boundary.spatial->steer.fifo_count>0,
        "probe fixture has a nonzero-time free-root steering continuation");
    const auto checkpoint=io::recording_json(runtime::spatial::CheckpointAccess::capture_bootstrap(ride));
    const auto publications=publication_bytes(ride);
    probe(ride);
    bt::require(checkpoint==io::recording_json(runtime::spatial::CheckpointAccess::capture_bootstrap(ride)) &&
        publications==publication_bytes(ride),
        "free-root probe preserves every live checkpoint and publication owner");
    bt::invalid([&]{(void)ride.probe_step_inputs(invalid,0.,0.);});
    bt::require(checkpoint==io::recording_json(runtime::spatial::CheckpointAccess::capture_bootstrap(ride)) &&
        publications==publication_bytes(ride),"rejected free-root probe is atomic on the same live owner");
    (void)ride.advance(64,drive,.2,.1,{});
    (void)reference->advance(64,drive,.2,.1,{});
    bt::require(ride.snapshot().step==64 && reference->snapshot().step==64 &&
        io::recording_json(runtime::spatial::CheckpointAccess::capture_bootstrap(ride))==
        io::recording_json(runtime::spatial::CheckpointAccess::capture_bootstrap(*reference)),
        "free-root probe leaves the next physical and accounting continuation bitwise unchanged");

    auto async_bootstrap=std::make_unique<runtime::RuntimeBootstrap>(*fixture->bootstrap);
    async_bootstrap->config.async_telemetry=true;
    auto asynchronous=std::make_unique<runtime::NativeRideRuntime>(*async_bootstrap);
    (void)asynchronous->advance(33,drive,.2,.1,{});
    bt::require(asynchronous->snapshot().step==33,"async probe fixture reaches a live partial period");
    const auto image=asynchronous->model_image();
    const auto physical_bytes=[&] {
        std::vector<std::byte> bytes;
        bike::validation::RuntimeRigAccess::inspect(*asynchronous,[&](const Stepper& stepper,const runtime::PhysicalStep& physical) {
            bytes=model_state(*stepper.model(),*stepper.data());
            append(bytes,bike::initialization::capture_current_state(stepper,physical,async_bootstrap->config,image));
        });
        return bytes;
    };
    const auto physics_before=physical_bytes();
    const auto async_publications=publication_bytes(*asynchronous);
    probe(*asynchronous);
    // The worker never mutates the main accounting owner until collect(); no
    // flush is allowed here, so a public probe cannot drain queued periods.
    bt::require(physics_before==physical_bytes() && async_publications==publication_bytes(*asynchronous),
        "async probes neither mutate physical owners nor collect or publish accounting");
    bt::invalid([&]{(void)asynchronous->probe_step_inputs(invalid,0.,0.);});
    bt::require(physics_before==physical_bytes() && async_publications==publication_bytes(*asynchronous),
        "rejected async probe preserves both the live physical and publication boundaries");
    bt::require(asynchronous->checkpoint_unavailable_reason()=="checkpoint requires synchronous accounting",
        "observational async probe preserves the source accounting mode");
}

double dot(const sp::Vec3& a,const sp::Vec3& b) {return std::inner_product(a.begin(),a.end(),b.begin(),0.);}
sp::Vec3 rz90(const sp::Vec3& value) {return {-value[1],value[0],value[2]};}
void close(const sp::Vec3& a,const sp::Vec3& b,double tolerance,std::string_view message) {
    for(std::size_t axis=0;axis<3;++axis)bt::require(std::abs(a[axis]-b[axis])<=tolerance,message);
}
void covariance() {
    const auto yaw=sp::Quat{std::cos(std::numbers::pi/4.),0.,0.,std::sin(std::numbers::pi/4.)};
    auto ground=std::make_shared<const bike::terrain::Terrain3dSource>(spatial_tests::plane_grid(.08,-.03,0.,17,17,.25,.25));
    auto rotated=std::make_shared<const bike::terrain::Terrain3dSource>(spatial_tests::plane_grid(.03,.08,0.,17,17,.25,.25));
    constexpr double radius=.3675,camber=.1;
    const auto grid=tyre::make_torus_grid({64,7,256},radius,.061);
    const tyre::CarcassSpec3d law{.pressure_pa=150000.,.k_area_n_per_m3=1e6,.maxwell_k_n_per_m3=2e5,
        .maxwell_tau_s=.02,.rim_clearance_m=.04,.rim_stiffness_n_per_m3=1e8,
        .constant_pressure=true,.provenance_status="synthetic"};
    for(double direction:{-1.,1.}) {
        tyre::WheelKinematics3d a;a.hub.position_world_m={1.5,-.5,.135+radius-.01};
        a.hub.quat_world_from_body_wxyz={std::cos(camber/2.),std::sin(camber/2.),0.,0.};
        a.hub.linear_velocity_world_mps={2.*direction,.1,-.2};a.hub.angular_velocity_world_rad_s={.1,.2,-.3};
        a.axle_world=sp::rotate_body_to_world(a.hub.quat_world_from_body_wxyz,{0.,1.,0.});a.relative_spin_rad_s=6.*direction;
        auto b=a;b.hub.position_world_m=rz90(a.hub.position_world_m);
        b.hub.quat_world_from_body_wxyz=sp::quat_multiply(yaw,a.hub.quat_world_from_body_wxyz);
        b.hub.linear_velocity_world_mps=rz90(a.hub.linear_velocity_world_mps);
        b.hub.angular_velocity_world_rad_s=rz90(a.hub.angular_velocity_world_rad_s);b.axle_world=rz90(a.axle_world);
        close(sp::rotate_world_to_body(a.hub.quat_world_from_body_wxyz,a.hub.linear_velocity_world_mps),
            sp::rotate_world_to_body(b.hub.quat_world_from_body_wxyz,b.hub.linear_velocity_world_mps),1e-9,
            "forward/reverse body-frame speed is covariant");
        std::vector<tyre::ContactElement3d> ea(grid.element_count()),eb(grid.element_count());
        bt::require(tyre::query_geometry(grid,a,ground->query_view(),ea)==CoreStatus::ok &&
            tyre::query_geometry(grid,b,rotated->query_view(),eb)==CoreStatus::ok,"rotated contact geometry is valid");
        std::vector<tyre::NormalElement3d> na(ea.size()),nb(eb.size());
        tyre::CarcassState3d ca;
        const auto material_count=static_cast<std::size_t>(grid.dims.n_theta_material)*grid.dims.n_phi;
        ca.deflection_m.assign(material_count,0.);ca.maxwell_q_n_per_m2.assign(material_count,0.);
        ca.previous_penetration_m.assign(material_count,0.);ca.active.assign(material_count,0);
        auto cb=ca;
        bt::require(tyre::update_carcass(law,ea,.001,false,ca,na)==CoreStatus::ok &&
            tyre::update_carcass(law,eb,.001,false,cb,nb)==CoreStatus::ok,"real normal kernels accept rotated contact");
        double load_a=0.,load_b=0.,power_a=0.,power_b=0.;
        for(std::size_t i=0;i<ea.size();++i) {
            bt::require(ea[i].valid==eb[i].valid && na[i].active==nb[i].active,"rotation preserves contact membership");
            if(!ea[i].valid)continue;
            close(rz90(ea[i].normal_world),eb[i].normal_world,1e-9,"support normal rotates with terrain");
            close(rz90(ea[i].point_world_m),eb[i].point_world_m,1e-9,"material contact point rotates with terrain");
            load_a+=na[i].normal_load_n;load_b+=nb[i].normal_load_n;
            sp::Vec3 fa{},fb{};
            for(std::size_t axis=0;axis<3;++axis){fa[axis]=na[i].normal_load_n*ea[i].normal_world[axis];fb[axis]=nb[i].normal_load_n*eb[i].normal_world[axis];}
            close(rz90(fa),fb,1e-9,"normal force response rotates with terrain");
            power_a+=dot(fa,sp::point_velocity_world(a.hub,ea[i].point_world_m));
            power_b+=dot(fb,sp::point_velocity_world(b.hub,eb[i].point_world_m));
        }
        bt::require(load_a>0. && std::abs(load_a-load_b)<=1e-9,"nonzero normal load response is rotation invariant");
        bt::require(std::abs(power_a-power_b)<=1e-9,"force power is invariant under rigid rotation");
        auto wa=a.hub.angular_velocity_world_rad_s,wb=b.hub.angular_velocity_world_rad_s;
        for(std::size_t axis=0;axis<3;++axis){wa[axis]+=a.relative_spin_rad_s*a.axle_world[axis];wb[axis]+=b.relative_spin_rad_s*b.axle_world[axis];}
        const double drive_a=7.5*sp::axis_rate(wa,a.hub.angular_velocity_world_rad_s,a.axle_world);
        const double drive_b=7.5*sp::axis_rate(wb,b.hub.angular_velocity_world_rad_s,b.axle_world);
        bt::require(std::abs(drive_a-drive_b)<=1e-9 && std::abs(drive_a)>1.,"internal drive power is nonzero and heading invariant");
    }
    const double pitch=std::numbers::pi/2.-5e-7;
    const auto attitude=sp::euler_zyx({std::cos(pitch/2.),0.,std::sin(pitch/2.),0.});
    bt::require(!attitude.valid && std::isfinite(attitude.roll_rad) && std::isfinite(attitude.pitch_rad) &&
        std::isfinite(attitude.yaw_rad),"near-singular attitude is explicitly invalid without NaNs");
}

void release_reporting() {
    for(const auto root:{sp::RootKind::constrained4,sp::RootKind::free6})
        for(const auto variant:{"lumped","articulated_planar"}) {
            auto request=spatial_tests::model_request(root);request->physics->spatial=request->spatial;
            if(std::string_view(variant)=="articulated_planar") {
                request->physics->articulated.pedal_attachment="spindle";
                request->physics->articulated.saddle_attachment="pin";
                request->physics->articulated.grip_attachment="connect";
            }
            cfg::post_init(*request->physics);cfg::RiderSpecs rider;rider.variant=variant;
            const auto report=runtime::implemented_capabilities(*request->physics,rider);
            bt::require(report.implemented && !report.supported && report.qualification_pending && report.rejections.empty(),
                "source-connected constrained/free owners still require independent numerical qualification");
        }
    auto extrusion=spatial_tests::model_request(sp::RootKind::free6);
    extrusion->spatial->terrain.kind=sp::TerrainRequestKind::profile_extrusion;
    extrusion->spatial->terrain.path="authored-profile.toml";
    extrusion->physics->spatial=extrusion->spatial;
    cfg::post_init(*extrusion->physics);
    const auto extrusion_report=runtime::implemented_capabilities(*extrusion->physics,cfg::RiderSpecs{});
    bt::require(extrusion_report.implemented && !extrusion_report.supported &&
        extrusion_report.qualification_pending && extrusion_report.rejections.empty(),
        "extrusion material dispatch is implemented while numerical qualification remains pending");
    auto unknown=spatial_tests::model_request(sp::RootKind::free6);
    unknown->physics->spatial=unknown->spatial;
    cfg::post_init(*unknown->physics);
    unknown->physics->drive.transmission_model="unknown_spatial_transmission";
    const cfg::RiderSpecs rider;
    const auto report=runtime::implemented_capabilities(*unknown->physics,rider);
    bt::require(!report.implemented && !report.supported && report.qualification_pending &&
        std::ranges::any_of(report.rejections,[](const std::string& reason) {
            return reason.starts_with("drive:unknown_spatial_transmission:");
        }),"unknown spatial transmission is reported as a closed dispatch row");
    invalid_path([&]{runtime::validate_physical_capabilities(*unknown->physics,rider);},
        "drive:unknown_spatial_transmission");
}
} // namespace

int main() {
    int failures=0;
    failures+=bt::run(topology_matrix);failures+=bt::run(identity_and_entrypoints);
    failures+=bt::run(producer_declaration_mismatch);
    failures+=bt::run(bootstrap_quaternion);failures+=bt::run(shoulder_layout);
    failures+=bt::run(public_probe_isolation);
    failures+=bt::run(covariance);failures+=bt::run(release_reporting);
    return failures?1:0;
}
