#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "spatial/support/recording_fixture.hpp"
#include "spatial/support/recording_mutation.hpp"
#include "replay/session.hpp"
#include "replay/playback.hpp"
#include "replay/spatial_recipe.hpp"
#include "runtime/setup_values.hpp"
#include "pneumatic3d/state.hpp"
#include "terrain/asset3d.hpp"
#include <array>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>

namespace {
namespace replay=bike::replay;
using namespace runtime::setupvalue;
void matching_generation(const replay::ReplaySession& session) {
    const auto& frame=session.render_frame();
    bt::require(frame.presentation.spatial.has_value(),"spatial replay publishes a spatial pose");
    bt::require(frame.presentation.spatial->generation==static_cast<std::uint64_t>(frame.state.generation),
        "snapshot and spatial pose share the replay generation");
    const auto view=bike::research::wire_value(runtime::Wire{frame.state.view});
    bt::require(bike::io::as_int64(field(field(field(view,"endpoint"),"spatial"),"generation"),"generation")==frame.state.generation,
        "wire presentation shares the replay generation");
}
void startup_provenance_cases(const std::filesystem::path& accepted,const std::filesystem::path& root) {
    using spatial_tests::saved_document;using spatial_tests::replace_document;
    const auto absent=root/"absent-initial-state.bin";
    bt::require(!std::filesystem::exists(absent),"replay startup fixture has no external state file");
    for(const bool imported:{false,true}) {
        const auto path=spatial_tests::mutate_recording(accepted,root/(imported?"imported-startup":"cached-startup"),[&](const auto& c){
            auto resolved=saved_document(c,"resolved_config.json");auto physics=field(resolved,"physics");
            set(physics,"equilibrium_cache_enabled",Value{true});
            if(imported)set(physics,"equilibrium_state_file",text(absent.string()));
            set(resolved,"physics",physics);
            // This fixture has no external profiles, so its canonical resolved
            // document already equals the content-normalized hash input.
            const auto& articulated=field(physics,"articulated");
            for(const auto key:{"joint_envelope_path","joint_strength_path","shoulder_profile"})
                if(const auto* value=find(articulated,key))bt::require(bike::io::is_null(*value),"startup fixture has no external profiles");
            const auto canonical=bike::io::canonical_json(resolved);
            const auto digest=bike::io::sha256(std::as_bytes(std::span{canonical}));
            const auto rows=c.query("SELECT summary_json FROM episodes WHERE episode_id=0");
            auto summary=bike::io::parse_recording_json(bike::io::as_string(field(Value{rows.at(0)},"summary_json"),"summary"));
            set(summary,"resolved_config",resolved);set(summary,"configuration_sha256",text(digest));
            const std::array params{text(bike::io::recording_json(summary))};
            c.execute("UPDATE episodes SET summary_json=? WHERE episode_id=0",params);
            auto recipe=saved_document(c,"recipe.json");set(recipe,"config_spatial",resolved);
            for(const auto key:{"checkpoint_initial","checkpoint_final"}) {
                auto checkpoint=field(recipe,key);auto identity=field(checkpoint,"identity");
                set(identity,"config_sha256",text(digest));set(checkpoint,"identity",identity);set(recipe,key,checkpoint);
                replace_document(c,std::string("spatial/")+key+".json",checkpoint);
            }
            replace_document(c,"recipe.json",recipe);replace_document(c,"resolved_config.json",resolved);
            replace_document(c,"spatial/config.json",resolved);
        });
        const auto result=replay::check(path,0);
        bt::require(result.matched,"embedded checkpoint replays with the original cache/import provenance and no external state file");
    }
}
void mismatch(const std::filesystem::path& path,std::string_view leaf) {
    const auto result=replay::check(path,0);
    bt::require(!result.matched && bike::io::as_string(field(result.details,"status"),"status")=="mismatch","changed evidence fails as a numerical/evidence mismatch");
    bt::require(bike::io::as_string(field(result.details,"first_mismatch_field"),"first path").find(leaf)!=std::string::npos,"corruption has a precise first mismatch field");
}
void corruption_cases(const std::filesystem::path& accepted,const std::filesystem::path& root) {
    using spatial_tests::mutate_recording;using spatial_tests::saved_document;using spatial_tests::replace_document;
    const auto command=mutate_recording(accepted,root/"command",[](const auto& c){
        c.execute("UPDATE commands_requested SET control=struct_update(control,human_torque_nm := control.human_torque_nm+1.0) WHERE episode_id=0 AND request_id=0");
    });
    // Actuator delay preserves the first window; command application is the
    // first deterministic divergence once the delayed request becomes due.
    const auto command_result=replay::check(command,0);
    bt::require(!command_result.matched && bike::io::as_string(field(command_result.details,"status"),"status")=="mismatch" &&
        !bike::io::is_null(field(command_result.details,"first_mismatch_field")),"changed requested torque fails through real delayed-control replay");
    const auto gyro=mutate_recording(accepted,root/"gyro",[](const auto& c){
        c.execute("UPDATE observations SET angular_rate_sensor_rad_s=[angular_rate_sensor_rad_s[1],angular_rate_sensor_rad_s[2]+1.0,angular_rate_sensor_rad_s[3]] WHERE episode_id=0 AND observation_id=1");
    });
    mismatch(gyro,"angular_rate_sensor_rad_s[1]");
    {
        replay::ReplaySession session(gyro);const auto initial=session.render_frame();bool failed=false;
        try { (void)session.advance(); } catch(const std::exception&) { failed=true; }
        bt::require(failed && session.error().has_value(),"first control window exposes its evidence mismatch");
        bt::require(session.snapshot().step==initial.state.step &&
            session.snapshot().integration_state==initial.state.integration_state &&
            session.render_frame().model_sha256==initial.model_sha256,
            "first-window mismatch rolls back to the complete verified startup frame");
        bt::require(session.snapshot().generation>initial.state.generation,"rollback publishes a new generation");
        matching_generation(session);
    }
    const auto bristle=mutate_recording(accepted,root/"bristle",[&](const auto& c){
        auto recipe=saved_document(c,"recipe.json");auto checkpoint=field(recipe,"checkpoint_final");
        auto bootstrap=field(checkpoint,"bootstrap2");auto tire=field(bootstrap,"tire");
        const auto bytes=bike::pneumatic3d::base64_decode_strict(bike::io::as_string(field(tire,"payload"),"payload"));
        bt::require(bytes.has_value(),"reference owns canonical tyre bytes");
        auto state=bike::pneumatic3d::decode_tyre_state(*bytes,bike::pneumatic3d::tyre_identity_from_wire(field(tire,"identity")));
        bt::require(state.has_value()&&!state->brush[0].bristle_ux_m.empty(),"reference has full material bristle state");
        state->brush[0].bristle_ux_m[0]+=.001;
        set(tire,"payload",text(bike::pneumatic3d::base64_canonical(bike::pneumatic3d::encode_tyre_state(*state))));
        set(bootstrap,"tire",tire);set(checkpoint,"bootstrap2",bootstrap);set(recipe,"checkpoint_final",checkpoint);
        replace_document(c,"recipe.json",recipe);replace_document(c,"spatial/checkpoint_final.json",checkpoint);
    });
    mismatch(bristle,"tire.brush[0].bristle_ux_m[0]");
    const auto calibration=mutate_recording(accepted,root/"calibration",[&](const auto& c){
        auto calibration=saved_document(c,"spatial/calibration.json");auto imu=field(calibration,"imu");
        auto angles=std::get<Array>(field(imu,"mount_rpy_rad").data);angles[0]=real(bike::io::as_double(angles[0],"mount roll")+.1);
        set(imu,"mount_rpy_rad",Value{std::move(angles)});set(calibration,"imu",imu);replace_document(c,"spatial/calibration.json",calibration);
    });
    const auto calibration_result=replay::check(calibration,0);
    bt::require(!calibration_result.matched && bike::io::as_string(field(calibration_result.details,"status"),"status")=="incompatible-execution" &&
        calibration_result.reason.find("calibration")!=std::string::npos,"changed calibration is incompatible before stepping");
    const auto terrain=mutate_recording(accepted,root/"terrain",[](const auto& c){
        auto bytes=spatial_tests::saved_asset(c,"spatial/terrain.terrain3d");
        bt::require(bytes.size()>bike::terrain::kTerrain3dHeaderBytes,"reference contains terrain cell bytes");
        bytes[bike::terrain::kTerrain3dHeaderBytes]^=std::byte{1};
        spatial_tests::replace_asset(c,"spatial/terrain.terrain3d",std::move(bytes));
    });
    const auto terrain_result=replay::check(terrain,0);
    bt::require(!terrain_result.matched && terrain_result.reason.find("terrain")!=std::string::npos &&
        bike::io::as_string(field(terrain_result.details,"status"),"status")!="incompatible-execution","terrain cell corruption fails input identity independently of execution compatibility");
    const auto execution=mutate_recording(accepted,root/"execution",[&](const auto& c){
        const auto rows=c.query("SELECT summary_json FROM episodes WHERE episode_id=0");
        auto summary=bike::io::parse_recording_json(bike::io::as_string(field(Value{rows.at(0)},"summary_json"),"summary"));
        auto identity=field(summary,"execution");auto digest=bike::io::as_string(field(identity,"execution_identity"),"execution identity");
        digest[0]=digest[0]=='0'?'1':'0';set(identity,"execution_identity",text(digest));set(summary,"execution",identity);
        auto research=field(summary,"research");set(research,"execution",identity);set(summary,"research",research);
        const std::array params{text(bike::io::recording_json(summary))};c.execute("UPDATE episodes SET summary_json=? WHERE episode_id=0",params);
        auto recipe=saved_document(c,"recipe.json");set(recipe,"execution_identity_sha256",text(digest));
        for(auto key:{"checkpoint_initial","checkpoint_final"}) {
            auto checkpoint=field(recipe,key);auto ident=field(checkpoint,"identity");set(ident,"solver",text(digest));set(checkpoint,"identity",ident);
            set(recipe,key,checkpoint);replace_document(c,std::string("spatial/")+key+".json",checkpoint);
        }
        replace_document(c,"recipe.json",recipe);
    });
    const auto incompatible=replay::check(execution,0);
    bt::require(!incompatible.matched && bike::io::as_string(field(incompatible.details,"status"),"status")=="incompatible-execution","different execution identity has a distinct result");
    replay::PlaybackSession inspection(execution,0);
    bt::require(inspection.frame(0.).presentation.spatial.has_value(),"incompatible checked execution remains inspectable from saved spatial truth");
}
}
int main(){return bt::run([]{
    bt::require(!replay::compare_evidence(integer(9007199254740993LL),integer(9007199254740992LL),"tick").matched,"large integer counters never round through double");
    bt::invalid([]{(void)replay::read_spatial_recipe(bike::io::parse_json(R"({"replay_contract_version":999})"));});
    auto fixture=spatial_tests::recording_fixture();bt::TemporaryDirectory root("spatial-checked-replay");
    const auto publication=bike::recording::finalize(*fixture.capture,root.path()/"accepted");
    {
        replay::ReplaySession session(publication.directory);while(!session.advance()){}
        bt::require(bike::io::as_bool(field(session.report(),"passed"),"passed"),"complete spatial episode checks under its own execution identity");
        bt::require(bike::io::as_int64(field(session.report(),"replay_contract_version"),"contract")==4,"spatial replay has its own version");
        matching_generation(session);
        const auto generation=session.snapshot().generation;session.restart();bt::require(session.snapshot().generation>generation,"restart publishes a new render generation");
        matching_generation(session);(void)session.advance(0.);matching_generation(session);
    }
    startup_provenance_cases(publication.directory,root.path());
    corruption_cases(publication.directory,root.path());
});}
