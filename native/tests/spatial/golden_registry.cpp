#include "support/migration_test.hpp"
#include "spatial/support/spatial_fixture.hpp"
#include "spatial/support/startup_cases.hpp"
#include "fixture_paths.hpp"
#include "golden/spatial_registry.hpp"
#include "golden/spatial_compare.hpp"
#include "golden/capture_testing.hpp"
#include "runtime/setup_values.hpp"
#include "support/temporary_directory.hpp"
#include "platform/files.hpp"
#include "pneumatic3d/state.hpp"
#include "research/environment.hpp"
#include "runtime/research_io.hpp"
#include "io/json.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <functional>
#include <filesystem>
namespace {
using namespace runtime::setupvalue;
namespace golden=bike::golden;
void rejects_with(const std::function<void()>& operation,std::string_view message) {
    bool rejected=false;try{operation();}catch(const std::invalid_argument& e){rejected=std::string_view(e.what()).find(message)!=std::string_view::npos;}
    bt::require(rejected,"negative reference reports its distinct failure class");
}
void write(const std::filesystem::path& path,std::string_view bytes) {bike::platform::write_exclusive(path,std::as_bytes(std::span(bytes)));}
Value synthetic_bootstrap(double elastic_energy=0.) {
    // Pure comparator fixture, never a recorded/accepted physical golden.
    bike::pneumatic3d::TyreState3d state;auto& id=state.identity;
    id.tier=bike::spatial::TyreTier::fast;id.dims=bike::pneumatic3d::tier_dims(id.tier);
    id.patch_capacity=id.dims.n_theta_fan*id.dims.n_phi;
    id.topology_fingerprint_hex=id.config_fingerprint_hex=id.terrain_sha256_hex=id.calibration_sha256_hex=std::string(64,'a');
    const auto material=static_cast<std::size_t>(id.dims.n_theta_material)*id.dims.n_phi;
    for(std::size_t i=0;i<2;++i) {
        auto& c=state.carcass[i];c.deflection_m.resize(material);c.maxwell_q_n_per_m2.resize(material);c.previous_penetration_m.resize(material);c.active.resize(material);
        auto& p=state.patches[i];p.capacity=id.patch_capacity;p.ids.resize(p.capacity);p.slot_used.resize(p.capacity);
        p.element_to_patch_slot.assign(id.patch_capacity,std::numeric_limits<std::uint32_t>::max());
        auto& b=state.brush[i];b.ux_m.resize(p.capacity);b.uy_m.resize(p.capacity);b.camber_state.resize(p.capacity);
        b.bristle_ux_m.resize(material);b.bristle_uy_m.resize(material);b.bristle_energy_j.resize(material);
    }
    state.brush[0].elastic_energy_j=elastic_energy;
    return Value{Object{{"tire",Value{Object{{"pneumatic3d_state",integer(1)},
        {"identity",bike::pneumatic3d::tyre_identity_to_wire(id)},{"encoding",text("base64")},
        {"payload",text(bike::pneumatic3d::base64_canonical(bike::pneumatic3d::encode_tyre_state(state)))}}}}}};
}
golden::EpisodeArtifact artifact(const Value& manifest) {
    golden::EpisodeArtifact result;result.manifest=manifest;result.channel_names={"x"};
    result.stored_rows={Value{Object{{"x",real(1.)}}}};result.rows={golden::flatten_row(result.stored_rows[0])};
    const auto empty=bike::io::double_array(std::span<const double>{},{0});
    result.state_qpos=result.state_qvel=result.state_act=result.state_warmstart=result.state_time=empty;
    result.ctrl_written=result.qfrc_applied=result.forces=result.tire_state=empty;
    const auto bootstrap=synthetic_bootstrap();set(result.manifest,"spatial_checkpoint_initial",bootstrap);set(result.manifest,"spatial_checkpoint_final",bootstrap);
    return result;
}
void nonfinite_comparison_cases(const golden::EpisodeArtifact& expected,
    const golden::EpisodeArtifact& actual,const Value& entry) {
    golden::testing::validate_spatial_numeric_evidence(runtime::WireObject{
        {"state",runtime::WireArray{0.,-0.,std::numeric_limits<double>::max(),true,nullptr}},
        {"labels",runtime::WireArray{"nan","inf","+inf","-inf"}}});
    for(const double nonfinite:{std::numeric_limits<double>::quiet_NaN(),
                               std::numeric_limits<double>::infinity(),
                               -std::numeric_limits<double>::infinity()}) {
        const runtime::Wire typed=runtime::WireObject{{"state",runtime::WireArray{0.,nonfinite}}};
        rejects_with([&]{golden::testing::validate_spatial_numeric_evidence(typed);},
            "physical_invalid: evidence.state[1]");
        auto invalid_reference=expected;auto invalid_same=expected;auto invalid_cross=actual;
        const auto numeric=bike::io::double_array(std::array{nonfinite},{1});
        invalid_reference.state_qpos=invalid_same.state_qpos=invalid_cross.state_qpos=numeric;
        for(const auto& [candidate,host]:std::array{
                std::pair{&invalid_same,"macos-arm64"},std::pair{&invalid_cross,"linux-aarch64"}}) {
            const auto errors=golden::compare_spatial_artifacts(invalid_reference,*candidate,entry,host);
            bt::require(!errors.empty()&&errors.front().starts_with("physical_invalid:"),
                "bit-identical nonfinite engine evidence never qualifies on either platform path");
        }
    }
    for(const auto* spelling:{"nan","+inf","-inf"}) {
        auto tagged_reference=expected,tagged_same=expected,tagged_cross=actual;
        const Value tagged{bike::io::Tagged{.kind="float64",.fields={{"value",spelling}}}};
        for(auto* candidate:{&tagged_reference,&tagged_same,&tagged_cross}) {
            candidate->channel_names.push_back("typed_numeric");
            set(candidate->stored_rows[0],"typed_numeric",tagged);
        }
        for(const auto& [candidate,host]:std::array{
                std::pair{&tagged_same,"macos-arm64"},std::pair{&tagged_cross,"linux-aarch64"}}) {
            const auto errors=golden::compare_spatial_artifacts(tagged_reference,*candidate,entry,host);
            bt::require(!errors.empty()&&errors.front().starts_with("physical_invalid:"),
                "typed nonfinite scalar evidence is invalid on same and cross platform paths");
        }
        set(tagged_reference.stored_rows[0],"typed_numeric",real(0.));
        const auto against_finite=golden::compare_spatial_artifacts(tagged_reference,tagged_same,entry,"macos-arm64");
        bt::require(!against_finite.empty() && against_finite.front().starts_with("physical_invalid:"),
            "a nonfinite candidate is invalid even when its reference is finite");
    }
    for(const auto* spelling:{"nan","inf","+inf","-inf"}) {
        auto text_reference=expected,text_same=expected,text_cross=actual;
        for(auto* candidate:{&text_reference,&text_same,&text_cross}) {
            candidate->channel_names.push_back("text_label");
            set(candidate->stored_rows[0],"text_label",text(spelling));
        }
        bt::require(golden::compare_spatial_artifacts(text_reference,text_same,entry,"macos-arm64").empty() &&
            golden::compare_spatial_artifacts(text_reference,text_cross,entry,"linux-aarch64").empty(),
            "literal string labels keep exact equality and are never guessed to be numeric NaN/Inf");
    }
}
void accepted_reference_checks(const Value& pending_entry) {
    bt::TemporaryDirectory root("spatial-golden-reference");
    auto entry=pending_entry;set(entry,"status",text("accepted"));
    for(auto key:{"config_sha256","terrain_sha256","calibration_sha256"})set(entry,key,text(std::string(64,'a')));
    set(entry,"provider",text("synthetic-reference"));set(entry,"expected_channels",Value{Array{text("x")}});
    write(root.path()/"model.mjb","synthetic-model-image");write(root.path()/"episode.npz","synthetic-comparator-input");
    set(entry,"model_sha256",text(bike::platform::file_sha256(root.path()/"model.mjb")));
    Value manifest{Object{{"spatial_platform",text("macos-arm64")},{"source_identity",text(std::string(64,'b'))},
        {"execution_identity",text(std::string(64,'c'))},{"provider",text("synthetic-reference")}}};
    for(auto key:{"config_sha256","model_sha256","terrain_sha256","calibration_sha256"})set(manifest,key,field(entry,key));
    write(root.path()/"manifest.json",bike::io::canonical_json(manifest));
    Object hashes;for(auto name:{"manifest.json","episode.npz","model.mjb"})hashes.emplace_back(name,text(bike::platform::file_sha256(root.path()/name)));
    Value platform{Object{{"status",text("accepted")},{"hashes",Value{hashes}},
        {"source_identity",text(std::string(64,'b'))},{"execution_identity",text(std::string(64,'c'))},
        {"provider",text("synthetic-reference")},{"executable",text("synthetic-comparison-fixture")}}};
    auto other=platform;set(other,"execution_identity",text(std::string(64,'d')));set(other,"provider",text("synthetic-candidate"));
    set(entry,"platforms",Value{Object{{"macos-arm64",platform},{"linux-aarch64",other}}});
    set(entry,"comparison",Value{Object{{"same_platform",text("bitwise")},{"cross_platform_tolerances",Value{Object{
        {"rows.*.x",Value{Object{{"atol",real(1e-6)},{"rtol",real(0.)}}}}}}}}});
    const Value registry{Object{{"spatial_manifest_version",integer(1)},{"entries",Value{Array{entry}}}}};
    golden::validate_spatial_manifest(registry);
    bt::require(!golden::inspect_spatial_reference(entry,root.path(),"linux-aarch64").pending,"complete synthetic reference is structurally inspectable");
    auto expected=artifact(manifest);auto actual=expected;
    bt::require(golden::compare_spatial_artifacts(expected,actual,entry,"macos-arm64").empty(),"same-platform identical typed evidence is bitwise equal");
    auto startup_changed=expected;set(startup_changed.manifest,"spatial_startup_candidate",synthetic_bootstrap(1e-6));
    bt::require(!golden::compare_spatial_artifacts(expected,startup_changed,entry,"macos-arm64").empty(),
        "reviewed comparison checks fresh startup material even when the replay checkpoint itself is unchanged");
    auto positive_zero=expected;auto negative_zero=expected;
    set(positive_zero.stored_rows[0],"x",real(0.));set(negative_zero.stored_rows[0],"x",real(-0.));
    bt::require(!golden::compare_spatial_artifacts(positive_zero,negative_zero,entry,"macos-arm64").empty(),"same-platform floating comparison preserves the zero sign bit");
    set(actual.manifest,"spatial_platform",text("linux-aarch64"));set(actual.manifest,"execution_identity",text(std::string(64,'d')));set(actual.manifest,"provider",text("synthetic-candidate"));
    set(actual.stored_rows[0],"x",real(1.+5e-7));
    bt::require(golden::compare_spatial_artifacts(expected,actual,entry,"linux-aarch64").empty(),"cross-platform floating field uses only its declared tolerance");
    auto changed=actual;set(changed.stored_rows[0],"x",real(1.+2e-6));
    bt::require(golden::compare_spatial_artifacts(expected,changed,entry,"linux-aarch64").front().starts_with("regression:"),"out-of-tolerance evidence is a regression");
    changed=actual;set(changed.manifest,"source_identity",text(std::string(64,'e')));
    rejects_with([&]{(void)golden::compare_spatial_artifacts(expected,changed,entry,"linux-aarch64");},"incompatibility: source_identity");
    changed=actual;set(changed.manifest,"calibration_sha256",text(std::string(64,'e')));
    rejects_with([&]{(void)golden::compare_spatial_artifacts(expected,changed,entry,"linux-aarch64");},"incompatibility: calibration");
    changed=actual;changed.first_failure=text("manufactured physical failure");
    bt::require(golden::compare_spatial_artifacts(expected,changed,entry,"linux-aarch64").front().starts_with("physical_invalid:"),"physical invalidity is distinct from a regression");
    nonfinite_comparison_cases(expected,actual,entry);
    std::filesystem::remove(root.path()/"episode.npz");
    rejects_with([&]{(void)golden::inspect_spatial_reference(entry,root.path(),"linux-aarch64");},"incompatibility: missing artifact");
    write(root.path()/"episode.npz","changed-artifact");
    rejects_with([&]{(void)golden::inspect_spatial_reference(entry,root.path(),"linux-aarch64");},"regression: artifact hash");
}
void complete_spatial_replay() {
    // Authored causal-state regression, never an accepted physical golden.
    // One startup supplies two different material/controller initial states
    // while preserving their configuration, geometry and identity.
    auto request=std::make_unique<bike::research::ResearchRequest>();
    request->setup=std::move(*spatial_tests::make_startup_request(
        bike::spatial::RootKind::free6,bike::spatial::TyreTier::detailed,false,2.));
    request->capture_mode=runtime::ResearchCaptureMode::full;
    request->experiment.duration_s=.1;request->experiment.control_period_s=.01;
    request->experiment.actuator_delay_s=.02;request->experiment.record_decimation=1;
    request->sensors_v2.latency_s=.02;request->sensors_v2.sample_period_s=.005;
    request->sensors_v2.maximum_read_period_s=.01;
    auto prepared=bike::research::prepare_environment(std::move(request));
    auto pristine=std::make_unique<runtime::ResearchBootstrap>(std::move(prepared->bootstrap));
    auto seeded=std::make_unique<runtime::ResearchBootstrap>(*pristine);
    for(auto& brush:seeded->ride.state.tire3d->brush) {
        std::ranges::fill(brush.bristle_uy_m,1e-6);
        std::ranges::fill(brush.bristle_energy_j,1e-12);
        brush.elastic_energy_j=1e-12*static_cast<double>(brush.bristle_energy_j.size());
    }
    auto& steer=seeded->ride.state.spatial->steer;
    std::ranges::fill(steer.delay_fifo_nm,.25);steer.fifo_count=steer.delay_fifo_nm.size();
    steer.work_j=.125;
    auto source=runtime::make_research(std::move(seeded));
    const auto reference=golden::capture_episode(*source,4,runtime::RideControl{});
    auto target=runtime::make_research(std::make_unique<runtime::ResearchBootstrap>(*pristine));
    const auto before=bike::io::canonical_json(target->checkpoint());
    for(const auto name:{"qpos","qacc_warmstart"}) {
        auto bad=reference;
        for(auto& [key,array]:bad.initial)if(key==name) {
            auto values=array.doubles();bt::require(!values.empty(),"initial consistency fixture has a mutable engine lane");
            values.front()+=.125;array=bike::io::double_array(values,{values.size()});
        }
        rejects_with([&]{(void)golden::replay_episode(*target,bad);},
            "initial "+std::string(name)+" disagrees with spatial checkpoint");
        bt::require(bike::io::canonical_json(target->checkpoint())==before,
            "contradictory initial arrays are rejected before reset or any owner publication");
    }
    auto invalid_material=reference;
    auto initial=field(invalid_material.manifest,"spatial_checkpoint_initial");
    auto tire=field(initial,"tire");set(tire,"payload",text("!"));set(initial,"tire",std::move(tire));
    set(invalid_material.manifest,"spatial_checkpoint_initial",std::move(initial));
    bt::invalid([&]{(void)golden::replay_episode(*target,invalid_material);});
    bt::require(bike::io::canonical_json(target->checkpoint())==before,
        "invalid spatial material is staged without changing the live target");
    const auto rows=golden::replay_episode(*target,reference);
    bt::require(golden::compare_rows(reference.rows,rows,0.,0.).empty(),
        "spatial replay restores reference material and controller history before the first force evaluation");
    const auto expected=source->snapshot(),actual=target->snapshot();
    bt::require(actual.generation==expected.generation && actual.step==expected.step &&
        spatial_tests::physics_state_bytes(actual)==spatial_tests::physics_state_bytes(expected),
        "the swapped runtime publishes the complete reference engine, tyre and causal continuation");
    bt::require(actual.first_failure.has_value()==expected.first_failure.has_value(),
        "post-replay observations read accounting from the new runtime owner");
    bt::require(golden::compare_episode(reference,*source,0.,0.).empty(),
        "matching seeded startup compares after capture on the same research host");
    bt::require(golden::compare_episode(reference,*source,0.,0.).empty(),
        "repeated comparison uses the reference epoch for fresh startup and replay");
    auto comparison=runtime::make_research(std::make_unique<runtime::ResearchBootstrap>(*pristine));
    const auto differences=golden::compare_episode(reference,*comparison,0.,0.);
    bt::require(std::ranges::find(differences,"regression: spatial_checkpoint_initial")!=differences.end(),
        "reference continuation cannot hide a changed freshly built material/controller startup");
    bt::require(spatial_tests::physics_state_bytes(comparison->snapshot())==spatial_tests::physics_state_bytes(expected),
        "startup diagnostics do not replace the consistent reference state used for the compared trajectory");
}
}
int main(){return bt::run([]{
    using namespace runtime::setupvalue;
    bt::invalid([]{bike::golden::validate_spatial_manifest(bike::io::parse_json(R"({"spatial_manifest_version":999})"));});
    bt::invalid([]{bike::golden::validate_spatial_manifest(bike::io::parse_json(R"({"spatial_manifest_version":1,"entries":[]})"));});
    const auto manifest=spatial_tests::json(std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"tests/fixtures/spatial3d/manifest.json");
    bike::golden::validate_spatial_manifest(manifest);
    const auto& pending_entry=bike::io::as_array(field(manifest,"entries"),"entries").front();
    bt::require(bike::golden::inspect_spatial_reference(pending_entry,"unused-pending-reference","macos-arm64").pending,"pending reference never claims successful comparison or reads nonexistent artifacts");
    accepted_reference_checks(pending_entry);
    complete_spatial_replay();
    auto bad=manifest;auto entries=std::get<Array>(field(bad,"entries").data);entries.push_back(entries.front());set(bad,"entries",Value{entries});
    bt::invalid([&]{bike::golden::validate_spatial_manifest(bad);});
    bad=manifest;entries=std::get<Array>(field(bad,"entries").data);set(entries.front(),"executable",text(""));set(bad,"entries",Value{entries});
    bt::invalid([&]{bike::golden::validate_spatial_manifest(bad);});
    bad=manifest;entries=std::get<Array>(field(bad,"entries").data);set(entries.front(),"model_sha256",text("tampered"));set(bad,"entries",Value{entries});
    bt::invalid([&]{bike::golden::validate_spatial_manifest(bad);});
    bad=manifest;entries=std::get<Array>(field(bad,"entries").data);auto versions=field(entries.front(),"schema_versions");set(versions,"sensor",integer(1));set(entries.front(),"schema_versions",versions);set(bad,"entries",Value{entries});
    bt::invalid([&]{bike::golden::validate_spatial_manifest(bad);});
});}
