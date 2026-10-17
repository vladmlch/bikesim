// RAM capture ownership/event domains. Not executed.
#include "support/migration_test.hpp"
#include "recording_fixture.hpp"
#include "platform/files.hpp"
#include <cstdlib>
#include <limits>

namespace {
void input_identity_contract() {
    namespace rec=bike::recording;namespace io=bike::io;namespace fs=std::filesystem;
    auto name=(fs::temp_directory_path()/"bike-t40-XXXXXX").string();
    std::vector<char> bytes(name.begin(),name.end());bytes.push_back('\0');
    if(!::mkdtemp(bytes.data()))throw std::runtime_error("test temporary directory unavailable");
    struct Cleanup { fs::path path; ~Cleanup(){std::error_code error;fs::remove_all(path,error);} } cleanup{fs::path(bytes.data())};
    const auto file=cleanup.path/"profile.json";
    const auto original=rec::json_asset("rider_joint_envelope.json",io::Value{io::Value::Object{{"revision",rec::value::integer(1)}}});
    bike::platform::write_exclusive(file,original.content);
    const io::Value metadata{io::Value::Object{{"resolved_config",io::Value{io::Value::Object{
        {"physics",io::Value{io::Value::Object{{"articulated",io::Value{io::Value::Object{
            {"joint_envelope_path",rec::value::text(file.string())}}}}}}},
        {"joint_envelope_sha256",rec::value::text(original.sha256)}}}}}};
    const auto status=[&]{const auto result=rec::input_identity_at_finalization(metadata);
        const auto& profile=rec::value::require(io::as_object(result,"identity"),"joint_envelope");
        return io::as_string(rec::value::require(io::as_object(profile,"profile"),"status"),"status");};
    bt::require(status()=="unchanged","captured profile identity compares current bytes against setup digest");
    fs::remove(file);
    const auto replacement=rec::json_asset("replacement.json",io::Value{io::Value::Object{{"revision",rec::value::integer(2)}}});
    bike::platform::write_exclusive(file,replacement.content);
    bt::require(status()=="changed"&&original.content!=replacement.content,"changed source never replaces captured asset bytes");
    fs::remove(file);
    bt::require(status()=="unavailable","missing current profile is recorded as a diagnostic, retaining setup identity");
}
}

int main(){return bt::run([]{
    namespace rec=bike::recording; namespace io=bike::io;
    auto metadata=recording_fixture::metadata();
    rec::RunCapture capture(metadata);
    auto source=std::make_shared<rec::EpisodeEvidence>(*recording_fixture::evidence(2));
    capture.add_episode(source); capture.add_episode(source);
    bt::require(capture.episodes().size()==1,"the same sealed owner is idempotent");
    const auto summary=io::recording_json(capture.episodes()[0]->summary);
    source->summary=io::Value{io::Value::Object{{"changed",io::Value{true}}}};
    source->assets[0].content.clear();
    bt::require(io::recording_json(capture.episodes()[0]->summary)==summary && !capture.episodes()[0]->assets[0].content.empty(),
        "caller mutation cannot change frozen summary/assets");
    auto different=std::make_shared<rec::EpisodeEvidence>(*source);
    bt::invalid([&]{capture.add_episode(different);});
    different->episode_id=2; bt::invalid([&]{capture.add_episode(different);});
    rec::EpisodeJournal journal(capture,0);
    (void)journal.emit("reset_requested",.5,2,3);
    (void)journal.emit("episode_ended",.5,3);
    bt::require(journal.next_id()==2&&capture.events().back().event_id==1,"journal preserves receive/apply boundary and contiguous identity");
    bt::invalid([&]{capture.event(capture.events().back());});
    auto invalid=capture.events().front(); invalid.event_id=2; invalid.applied_step=1;
    bt::invalid([&]{capture.event(invalid);});
    auto disabled=recording_fixture::capture(100,false);
    bt::require(disabled->episodes().empty()&&disabled->events().empty(),"opt-out capture keeps no optional growing evidence");
    const auto before=disabled->metadata().run_uuid;
    bt::require(!before.empty()&&disabled->state()==rec::RunCapture::State::collecting,"capture only owns RAM before finalization");
    rec::ForceSample force(0.,{1.},{2.},{{"chain",{3.}}});
    bt::require(force.component_powers()==std::vector<std::pair<std::string,double>>{{"chain",6.}},"force sample uses the incoming velocity");
    bt::invalid([]{rec::ForceSample bad(0.,{1.},{2.},{{"chain",{3.,4.}}});});
    bt::invalid([]{rec::ForceSample bad(0.,{1.},{std::numeric_limits<double>::infinity()},{});});
    metadata.run_uuid="bad"; bt::invalid([&]{metadata.validate();});
    input_identity_contract();
});}
