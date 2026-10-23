// Reports, lossless explicit export and legacy bundle integrity. Not run.
#include "diag.hpp"
#include "recording_reader_test_support.hpp"
#include "recording_fixture.hpp"
#include "recording/reader.hpp"
#include "recording/report.hpp"
#include "recording/export.hpp"
#include "replay/export.hpp"
#include "io/hash.hpp"
#include <limits>

namespace {
namespace rec=bike::recording;
namespace io=bike::io;
using namespace rec::detail;
void reports_and_exports() {
    recording_reader_test::Directory tmp;
    const auto publication=recording_fixture::publish(tmp.path/"runs",3); rec::Recording reader(publication.directory);
    const auto report=rec::render_summary(reader);
    for(auto heading:{"# Run recording","## Episode 0","Scalar coverage and extrema","Recorded component power integrals","Field and coordinate coverage"})
        bt::require(report.find(heading)!=std::string::npos,"automatic report includes source evidence/coverage sections");
    bt::require(report.find("9007199254740993")!=std::string::npos&&report.find("Missing")!=std::string::npos&&report.find("Nonfinite")!=std::string::npos,
        "report preserves large integers and exposes missing/nonfinite coverage");
    bt::require(report==bt::text(publication.directory/"summary.md"),"publication report and explicit rendering share the same implementation");
    const auto jsonl=tmp.path/"metrics.jsonl",csv=tmp.path/"metrics.csv";
    const auto exported=rec::export_data(reader,0,"metrics","jsonl",jsonl);
    bt::require(count(field(exported,"rows"))==3,"default explicit export is unlimited");
    const auto bytes=bt::text(jsonl);
    bt::require(bytes.find("9007199254740993")!=std::string::npos&&bytes.find("\\nUTF-8")!=std::string::npos,"JSONL keeps exact integer and escaped structured string content");
    (void)rec::export_data(reader,0,"metrics","csv",csv);
    const auto text=bt::text(csv);
    bt::require(text.find("\r\n")!=std::string::npos&&text.find("quote,\"\" and\nUTF-8: велосипед")!=std::string::npos,"CSV uses CRLF rows and quoted embedded quote/newline/UTF-8 content");
    recording_reader_test::rejects([&]{(void)rec::export_data(reader,0,"metrics","csv",csv);});
    bt::invalid([&]{(void)rec::export_data(reader,0,"physics","jsonl",tmp.path/"wrong.jsonl");});
    bt::invalid([&]{(void)rec::export_data(reader,0,"metrics","parquet",tmp.path/"wrong.parquet");});
    const auto limited=rec::export_data(reader,0,"metrics","jsonl",tmp.path/"limited.jsonl",1);
    bt::require(count(field(limited,"rows"))==1,"explicit export limit applies only when requested");
}
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
void legacy_integrity() {
    namespace replay=bike::replay;
    recording_reader_test::Directory tmp;
    const auto hash=text(std::string(64,'a'));
    const Value execution{Object{{"backend",text("native")},{"native_source_sha256",hash},{"execution_identity",hash},
        {"mujoco_library_sha256",hash},{"numeric_provider",text("synthetic test identity")},{"executable_sha256",hash},{"source_identity_scope",text("test only")}}};
    replay::LegacySnapshot snapshot;
    snapshot.data.manifest=Value{Object{{"execution",execution},{"start_x_m",real(2.)},{"heightfield",Value{Object{}}}}};
    const Value articulated{Object{{"joint_envelope_path",Value{}},{"joint_strength_path",Value{}}}};
    const Value physical{Object{{"articulated",articulated}}};
    snapshot.data.summary=Value{Object{{"research",Value{Object{{"execution",execution}}}},
        {"resolved_config",Value{Object{{"physics",physical}}}}}};
    snapshot.data.metrics=Value{Object{{"exact_integer",integer(9007199254740993LL)}}};
    snapshot.data.initial_state={0.,-0.,1.}; snapshot.data.final_state={.01,-0.,2.}; snapshot.data.observations={Value{Object{}}};
    snapshot.data.assets["terrain_vertices.npy"]=io::encode_npy(io::double_array(std::array{0.,0.,1.,0.},{2,2}));
    const std::string track="name = \"synthetic fixture\"\nlength_m = 1.0\n";
    const auto track_bytes=std::as_bytes(std::span(track.data(),track.size())); snapshot.data.assets["track.toml"]={track_bytes.begin(),track_bytes.end()};
    snapshot.columns={{"time_s",{0.,.01}},{"optional",{std::numeric_limits<double>::quiet_NaN(),2.}}};
    const auto destination=tmp.path/"legacy"; const auto result=replay::export_legacy(snapshot,destination);
    bt::require(!io::as_bool(field(field(result,"compatibility"),"historical_python_reader_accepts_execution"),"compatibility"),"native executable identity is not relabeled as an old extension execution");
    const auto loaded=replay::load_legacy_input(destination);
    bt::require(loaded.initial_state==snapshot.data.initial_state&&loaded.final_state==snapshot.data.final_state,
        "NPZ fixed members round trip through the CRC-validating NumPy reader");
    bt::require(equal(field(loaded.manifest,"execution"),execution)&&io::as_object(field(loaded.manifest,"file_sha256"),"hashes").size()==12,
        "legacy fixed file set and actual payload hashes preserve provenance");
    bt::require(bt::text(destination/"trace.csv")=="sensor_time_s\r\n"&&bt::text(destination/"telemetry.csv").find(",0.0\r\n")!=std::string::npos,
        "empty trace header and explicit NaN padding follow the legacy numeric view");
    recording_reader_test::rejects([&]{(void)replay::export_legacy(snapshot,destination);});
    std::filesystem::remove(destination/"trace.csv"); recording_reader_test::write(destination/"trace.csv","tampered\n");
    bt::invalid([&]{(void)replay::load_legacy_input(destination);});
    (void)replay::export_legacy(snapshot,destination,true);
    bt::require(replay::load_legacy_input(destination).replay_contract_version==2,"explicit overwrite republishes validated payloads with manifest last");
    auto nonfinite=snapshot; set(nonfinite.data.summary,"invalid",real(std::numeric_limits<double>::infinity()));
    bt::invalid([&]{(void)replay::export_legacy(nonfinite,tmp.path/"nonfinite");});
    bt::require(!std::filesystem::exists(tmp.path/"nonfinite"/"replay.json"),"a failed legacy projection never publishes its readiness manifest");
}
NATIVE_DIAG_POP
}
int main(){return bt::run([]{reports_and_exports();legacy_integrity();});}
