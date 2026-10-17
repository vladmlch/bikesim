// Exclusive final publication, retry/crash boundaries and F145 worker. Unrun.
#include "support/migration_test.hpp"
#include "recording_fixture.hpp"
#include "async_close_contract.hpp"
#include "recording/reader.hpp"
#include "research/lifecycle.hpp"
#include "platform/files.hpp"
#include <array>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {
namespace rec=bike::recording;namespace fs=std::filesystem;namespace io=bike::io;
struct Temporary {
    fs::path path;
    Temporary(){auto name=(fs::temp_directory_path()/"bike-t42-XXXXXX").string();
        std::vector<char> bytes(name.begin(),name.end());bytes.push_back('\0');
        if(!::mkdtemp(bytes.data()))throw std::runtime_error("temporary directory unavailable");path=bytes.data();}
    ~Temporary(){std::error_code error;fs::remove_all(path,error);}
};
rec::FinalizeOptions options(){rec::FinalizeOptions out;out.report=[](const fs::path&){return std::string("# Authored report\n");};return out;}
std::vector<fs::path> manifests(const fs::path& root){
    std::vector<fs::path> result;
    for(const auto& file:fs::recursive_directory_iterator(root))
        if(file.path().filename()=="manifest.json")result.push_back(file.path());
    return result;
}
void fault_matrix(){
    const std::array stages{"database_open","schema_write","metadata_write","catalog_write","append","chunk_append",
        "appender_flush","appender_close","appender_destroy","views_write","validate_database","commit","checkpoint",
        "database_close","database_link","sync_run.duckdb","hash_run.duckdb","sync_summary.md","hash_summary.md",
        "manifest_write","destination_create","link_run.duckdb","link_summary.md","manifest_link"};
    for(const auto* stage:stages){
        Temporary temp;auto capture=recording_fixture::capture(1);auto settings=options();
        settings.before_stage=[stage](std::string_view current){if(current==stage)throw std::runtime_error("injected publication failure");};
        bool rejected=false;try{(void)rec::finalize(*capture,temp.path,settings);}catch(const std::runtime_error&){rejected=true;}
        bt::require(rejected&&capture->state()==rec::RunCapture::State::publication_failed,
            std::string("failure must retain retryable state at ")+stage);
        bt::require(manifests(temp.path).empty()&&capture->episodes()[0]->physical->row_count()==1,
            "a precommit failure cannot seal partial data or discard original RAM");
        const auto success=rec::finalize(*capture,temp.path,options());
        const auto repeated=rec::finalize(*capture,temp.path/"ignored",options());
        bt::require(success.directory==repeated.directory&&manifests(temp.path).size()==1,"retry chooses a new exclusive destination and success is idempotent");
        const auto opened=rec::open_recording(success.directory);
        bt::require(opened!=nullptr,"retry validates with the read-only reader");
    }
}
void publication_contract(){
    Temporary temp;
    auto off=recording_fixture::capture(1,false);
    const auto disabled=rec::finalize(*off,temp.path/"disabled",options());
    bt::require(disabled.report_status=="disabled"&&!fs::exists(temp.path/"disabled"),"record-off creates no output path");
    auto capture=recording_fixture::capture(1);auto settings=options();
    settings.report=[](const fs::path&)->std::string{throw std::runtime_error("report renderer failure");};
    const auto report_failed=rec::finalize(*capture,temp.path,settings);
    bt::require(report_failed.report_status=="failed"&&!fs::exists(report_failed.directory/"summary.md"),"report failure can seal a valid DB with explicit status");
    const auto manifest=io::parse_recording_json(bt::text(report_failed.directory/"manifest.json"));
    bt::require(!io::is_null(rec::value::require(manifest,"report_error")),"report failure detail is recorded");
    bt::require(io::as_string(rec::value::require(rec::value::require(manifest,"file_sha256"),"run.duckdb"),"hash")==
        bike::platform::file_sha256(report_failed.directory/"run.duckdb"),"manifest hashes the closed payload");
    auto first=recording_fixture::capture(1),second=recording_fixture::capture(1);
    rec::Publication a,b;std::exception_ptr left_error,right_error;
    std::thread left([&]{try{a=rec::finalize(*first,temp.path,options());}catch(...){left_error=std::current_exception();}});
    std::thread right([&]{try{b=rec::finalize(*second,temp.path,options());}catch(...){right_error=std::current_exception();}});
    left.join();right.join();if(left_error)std::rethrow_exception(left_error);if(right_error)std::rethrow_exception(right_error);
    bt::require(a.directory!=b.directory&&fs::exists(a.directory/"manifest.json")&&fs::exists(b.directory/"manifest.json"),
        "concurrent finalizers with identical launch identity never share a destination");
    auto retry=recording_fixture::capture(1);const auto base=temp.path/"20261017T120000.123456Z-00000000-0000-4000-8000-000000000038";
    // base already exists from report_failed; append the next known-free name
    // as a symlink so the exclusive creator must skip it too.
    const auto occupied=temp.path/(base.filename().string()+"-3");
    fs::create_directory_symlink(report_failed.directory,occupied);
    const auto after=rec::finalize(*retry,temp.path,options());
    bt::require(after.directory!=base&&after.directory!=occupied&&fs::is_symlink(occupied),"existing and symlink destinations remain unadopted");
}
void crash_and_signal_contract(){
    for(const auto* stage:{"checkpoint","link_run.duckdb","manifest_link"}){
        Temporary temp;const auto pid=::fork();if(pid<0)throw std::runtime_error("fork failed");
        if(pid==0){auto capture=recording_fixture::capture(1);auto settings=options();
            settings.before_stage=[stage](std::string_view current){if(current==stage){::raise(SIGKILL);::_exit(99);}};
            try{(void)rec::finalize(*capture,temp.path,settings);}catch(...){::_exit(98);}::_exit(97);}
        int status=0;if(::waitpid(pid,&status,0)!=pid)throw std::runtime_error("waitpid failed");
        bt::require(WIFSIGNALED(status)&&WTERMSIG(status)==SIGKILL&&manifests(temp.path).empty(),"killed precommit finalizer leaves no recognizable sealed partial recording");
    }
    Temporary temp;const auto pid=::fork();if(pid<0)throw std::runtime_error("fork failed");
    if(pid==0){auto capture=recording_fixture::capture(1);auto settings=options();
        settings.before_stage=[](std::string_view stage){if(stage=="manifest_link"){::raise(SIGINT);::raise(SIGINT);}};
        try{(void)rec::finalize_guarded(*capture,temp.path,settings);}catch(const bike::research::OperatorInterrupted&){::_exit(capture->state()==rec::RunCapture::State::published?0:93);}catch(...){::_exit(94);}::_exit(95);}
    int status=0;if(::waitpid(pid,&status,0)!=pid)throw std::runtime_error("waitpid failed");
    bt::require(WIFEXITED(status)&&WEXITSTATUS(status)==0&&manifests(temp.path).size()==1,
        "repeated SIGINT defers until the owning finalizer seals and caches its path");
}
}
int main(){return bt::run([]{async_close_contract::run();fault_matrix();publication_contract();crash_and_signal_contract();});}
