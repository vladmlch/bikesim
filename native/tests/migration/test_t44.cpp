// Real native record/replay plus presentation-only cases. Authored, not run.
#include "recording_reader_test_support.hpp"
#include "recording_fixture.hpp"
#include "replay/session.hpp"
#include "replay/playback.hpp"
#include "replay/export.hpp"
#include "replay/rebuild.hpp"
#include "research/evidence.hpp"
#include "terrain/profile.hpp"
#include "engine_call.hpp"
#include <cmath>
#include <limits>

namespace {
namespace io = bike::io;
namespace replay = bike::replay;
namespace rec = bike::recording;
using namespace rec::detail;
void comparisons_and_playback() {
    bt::require(replay::compare_evidence(io::parse_json("{\"v\":[1.0,null,true]}"),io::parse_json("{\"v\":[1.0000000001,null,true]}" )).matched,
        "checked replay recursively compares finite numbers with the documented tolerance");
    const auto bad = replay::compare_evidence(io::parse_json("{\"v\":[1.0,2.0]}"),io::parse_json("{\"v\":[1.0,3.0]}"));
    bt::require(!bad.matched && bad.first_path == "evidence.v[1]" && bad.max_absolute_error == 1., "first mismatch is a precise leaf with maximum numerical error");
    bt::require(!replay::compare_evidence(Value{true},integer(1)).matched, "bool cannot pass as an integer command");
    const auto invalid = real(std::numeric_limits<double>::quiet_NaN());
    bt::require(!replay::compare_evidence(invalid,invalid).matched, "matching nonfinite sentinels never certify checked replay");
    replay::PlaybackData data;
    data.times_s={0.,1.}; data.ends_s={1.,2.}; data.duration_s=2.;
    data.qpos={{0.,1.,0.,0.,0.},{2.,-1.,0.,0.,0.}}; data.qvel={{1.},{3.}}; data.quaternion_starts={1};
    replay::validate_playback(data,5,1);
    const auto pose = replay::pose_at(data,.5);
    bt::require(pose[0]==1. && pose[1]==1. && pose[2]==0., "pose interpolation normalizes antipodal equivalent quaternion endpoints safely");
    bt::require(replay::pose_at(data,-1.)[0]==0. && replay::pose_at(data,3.)[0]==2., "playback clamps to recorded endpoints without integration");
    bt::invalid([&] { (void)replay::pose_at(data,std::numeric_limits<double>::quiet_NaN()); });
    auto changed=data; changed.times_s[1]=0.; bt::invalid([&] { replay::validate_playback(changed,5,1); });
    changed=data; changed.qvel[1].clear(); bt::invalid([&] { replay::validate_playback(changed,5,1); });
    changed=data; changed.qpos[1][0]=std::numeric_limits<double>::infinity(); bt::invalid([&] { replay::validate_playback(changed,5,1); });
    replay::ReplayInput historical; historical.replay_contract_version=1;
    bool incompatible=false; try { replay::check_execution(historical); } catch(const replay::IncompatibleExecution&) { incompatible=true; }
    bt::require(incompatible,"historical arithmetic is rejected explicitly until qualified, never relabeled as native-executable replay");
}
void native_roundtrip() {
    namespace research=bike::research;
    research::ResearchRequest request;
    bike::configuration::SimulationPhysicsConfig physics;
    physics.physics_mode="physical"; physics.drive_mode="crank_effort"; physics.timestep_s=.0005;
    physics.initial_speed_mps=.1; physics.equilibrium_cache_enabled=false;
    physics.tires.backend="compliant_2d"; physics.tires.surface_mode="track";
    bike::configuration::RiderSpecs rider; rider.variant="lumped";
    request.setup.model.physics=physics; request.setup.model.rider=rider; request.setup.model.include_rider=true;
    request.setup.model.mode="ride"; request.setup.model.crank_joint=true; request.setup.track=bike::terrain::flat(12.);
    request.experiment.duration_s=.006; request.experiment.control_period_s=.002; request.experiment.actuator_delay_s=.001;
    request.experiment.record_decimation=1; request.experiment.stop_on_model_violation=false;
    request.sensors=research::SensorConfig::ideal(); request.sensors.sample_period_s=.001;
    request.road_resolution_m=.05; request.capture_mode=runtime::ResearchCaptureMode::full;
    auto prepared=research::prepare_environment(request);
    bike::model::ModelOwner model{engine::load_model_image(prepared.bootstrap.ride.model.mjb)};
    const auto catalog=rec::build_catalog(*model);
    auto environment=runtime::make_research(std::move(prepared.bootstrap));
    runtime::RideControl command; command.motor_torque_nm=0.; command.human_torque_nm=0.;
    for (;;) {
        const auto status=research::wire_value(runtime::Wire{environment->status()});
        if(io::as_bool(field(status,"terminated"),"terminated")||io::as_bool(field(status,"truncated"),"truncated"))break;
        environment->begin_control(command,0.,0.);
        bt::require(environment->advance_control({}).has_value(),"unbounded native control completes a real physical window");
        environment->acknowledge_control();
    }
    auto legacy=replay::capture_legacy(*environment);
    auto owner=environment->seal_episode();
    set(owner.run_metadata,"identity_checks",research::identity_receipt(owner.configuration.metadata));
    const auto summary=research::summarize(owner);
    bt::require(io::as_bool(field(field(summary,"recording_extents"),"replay_eligible"),"eligible"),"clean finite completed recording qualifies structurally");
    recording_reader_test::Directory tmp;
    rec::RunCapture run(recording_fixture::metadata("00000000-0000-4000-8000-000000000044"));
    run.add_episode(rec::make_research_evidence(0,std::move(owner),catalog,summary));
    const auto publication=rec::finalize(run,tmp.path / "runs"); environment->close();
    replay::ReplaySession session(publication.directory,0);
    const auto initial=session.snapshot();
    bt::require(!session.advance(0.,0)&&session.pending()&&session.snapshot().step==initial.step,"zero-budget yield keeps one accepted pending control without integrating");
    session.close();
    bt::require(!io::as_bool(field(session.diagnostics(),"passed"),"passed"),"early close never completes unverified replay");
    recording_reader_test::rejects([&] { (void)session.report(); });
    replay::ReplaySession completed(publication.directory,0);
    while(!completed.advance(.008)){}
    bt::require(completed.done()&&io::as_bool(field(completed.report(),"passed"),"passed"),"saved evidence passes only after actual transitions, observations, metrics, outcome and final state agree");
    const auto generation=completed.snapshot().generation; completed.restart();
    bt::require(completed.snapshot().step==0&&completed.snapshot().generation>generation&&!completed.done(),"restart reconstructs initial verified state and starts a new generation");
    completed.close();
    replay::PlaybackSession playback(publication.directory,0);
    const auto rendered=playback.frame(.001,2);
    bt::require(rendered.state.generation==2&&rendered.state.time_s==.001&&rendered.model_sha256==playback.model_image().sha256,
        "saved-state playback produces owned model-bound frames without a live replay control loop");
    bt::require(!legacy.data.transitions.empty(),"test includes actual transitions");
    auto& transition=legacy.data.transitions.front();
    set(transition,"numerically_valid",Value{!io::as_bool(field(transition,"numerically_valid"),"numerical")});
    const auto tampered=tmp.path/"changed-evidence"; (void)replay::export_legacy(legacy,tampered);
    replay::ReplaySession failing(tampered,0);
    recording_reader_test::rejects([&] { while(!failing.advance(.008)){} });
    const auto diagnostic=failing.diagnostics();
    bt::require(!failing.done()&&failing.error().has_value()&&!io::is_null(field(diagnostic,"first_mismatch_field"))&&
        !io::as_bool(field(diagnostic,"passed"),"passed"),"real runtime rejects altered transition evidence with structured first mismatch");
    bt::require(failing.snapshot().step==0&&failing.snapshot().generation>0,"failed verification restores the last verified frame under a newer presentation generation");
    failing.close();
}
}
int main(){return bt::run([]{comparisons_and_playback();native_roundtrip();});}
