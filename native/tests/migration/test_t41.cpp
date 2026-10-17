// Relational validation and typed persistence. Source only; not executed.
#include "support/migration_test.hpp"
#include "recording_fixture.hpp"
#include "recording/database.hpp"
#include "platform/files.hpp"
#include <cstdlib>

namespace {
namespace rec=bike::recording; namespace io=bike::io; namespace fs=std::filesystem;
struct Temporary {
    fs::path path;
    Temporary(){auto name=(fs::temp_directory_path()/"bike-t41-XXXXXX").string();
        std::vector<char> bytes(name.begin(),name.end());bytes.push_back('\0');
        if(!::mkdtemp(bytes.data()))throw std::runtime_error("test temporary directory unavailable");path=bytes.data();}
    ~Temporary(){std::error_code error;fs::remove_all(path,error);}
};
void database_contract(){
    Temporary temp;auto run=recording_fixture::capture(5);const auto file=temp.path/"run.duckdb";
    rec::write_database(file,*run);
    bt::require(fs::is_regular_file(file)&&!fs::exists(file.string()+".wal"),"publication DB is closed and checkpointed");
    {
        rec::Database database(file,true);auto& c=database.connection();
        const auto counts=rec::validate_database(c);
        bt::require(io::as_int64(rec::value::require(counts,"physics"),"physics")==5,"physical extent round-trips");
        for(const auto& table:rec::schema_catalog().table_order())if(table.name.starts_with("channel_"))
            bt::require(io::as_int64(rec::value::require(counts,table.name),table.name)==5,"all absent channel roots still cover five intervals");
        const auto integer=c.query("SELECT bigint_value FROM metrics WHERE name='iterations'");
        bt::require(io::as_int64(rec::value::require(integer.at(0),"bigint_value"),"value")==9007199254740993LL,"metric integer is never rounded through double");
        const auto parent=c.query("SELECT \"__energy_present\" AS present, \"__energy_valid\" AS valid FROM channel_energy ORDER BY interval_id");
        bt::require(!io::as_bool(rec::value::require(parent.at(0),"present"),"present")&&!io::as_bool(rec::value::require(parent.at(0),"valid"),"valid"),"absent root has explicit false masks");
        const auto assets=c.query("SELECT content FROM assets");
        bt::require(std::holds_alternative<io::Bytes>(rec::value::require(assets.at(0),"content").data),"asset bytes are a BLOB");
    }
    const auto hash=bike::platform::file_sha256(file);
    bool refused=false;try{rec::write_database(file,*run);}catch(const std::system_error&){refused=true;}
    bt::require(refused&&bike::platform::file_sha256(file)==hash,"an existing database is never reopened/adopted");
    {
        rec::Database database(file,false);
        database.connection().execute("UPDATE channel_energy SET \"__energy_valid\"=TRUE WHERE interval_id=0");
        bt::invalid([&]{(void)rec::validate_database(database.connection());});
    }
}
void corrupt_evidence(){
    Temporary temp;
    auto evidence=std::make_shared<rec::EpisodeEvidence>(*recording_fixture::evidence(1));
    evidence->assets.front().content.push_back(std::byte{42});
    rec::RunCapture capture(recording_fixture::metadata());capture.add_episode(evidence);
    bt::invalid([&]{rec::write_database(temp.path/"bad.duckdb",capture);});
    bt::require(!fs::exists(temp.path/"bad.duckdb"),"hash failure cannot publish a database");
    auto empty=recording_fixture::capture(0);
    rec::write_database(temp.path/"empty.duckdb",*empty);
    rec::Database database(temp.path/"empty.duckdb",true);
    const auto counts=rec::validate_database(database.connection());
    bt::require(io::as_int64(rec::value::require(counts,"physics"),"physics")==0,"empty prefix persists without fabricated rows");
}
void empty_routed_root(){
    Temporary temp;auto episode=recording_fixture::episode(1);
    auto sample=std::make_shared<runtime::PhysicalSampleData>(*episode.samples[0]);
    sample->channels={{"component_work_j",runtime::WireObject{}},{"rider_control",runtime::WireObject{}},
        {"rider_intent",runtime::WireObject{}},
        {"suspension",runtime::WireObject{{"fork_travel_m",0.},{"shock_stroke_m",0.},
            {"fork_velocity_mps",0.},{"shock_velocity_mps",0.},{"shock_solver_limit_power_w",0.},
            {"linkage_closure_max_m",0.},{"generalized_force_components_n",runtime::WireObject{}}}}};episode.samples[0]=sample;
    auto evidence=std::make_shared<rec::EpisodeEvidence>(*recording_fixture::evidence(1));
    evidence->physical=rec::make_physical_source(std::move(episode),recording_fixture::catalog());
    rec::RunCapture capture(recording_fixture::metadata());capture.add_episode(evidence);
    rec::write_database(temp.path/"empty_roots.duckdb",capture);
    rec::Database database(temp.path/"empty_roots.duckdb",false);
    (void)rec::validate_database(database.connection());
    const auto rows=database.connection().query("SELECT rider_intent_present AS leaf, \"__rider_intent_present\" AS present, \"__rider_intent_valid\" AS valid, rider_intent IS NULL AS absent FROM channel_rider");
    bt::require(rows.size()==1&&!io::as_bool(rec::value::require(rows[0],"leaf"),"leaf")&&
        io::as_bool(rec::value::require(rows[0],"present"),"present")&&io::as_bool(rec::value::require(rows[0],"valid"),"valid")&&
        io::as_bool(rec::value::require(rows[0],"absent"),"absent"),"source empty root retains (present=true,valid=true,leaf_present=false,NULL) exactly");
    const auto nested=database.connection().query("SELECT len(generalized_force_components_n) AS size, generalized_force_components_n IS NOT NULL AS present FROM channel_suspension");
    bt::require(io::as_int64(rec::value::require(nested[0],"size"),"size")==0&&io::as_bool(rec::value::require(nested[0],"present"),"present"),
        "a deeper empty map remains a valid empty typed container without child reads");
    database.connection().execute("UPDATE channel_rider SET \"__rider_intent_valid\"=FALSE");
    bt::invalid([&]{(void)rec::validate_database(database.connection());});
}
}
int main(){return bt::run([]{database_contract();corrupt_evidence();empty_routed_root();});}
