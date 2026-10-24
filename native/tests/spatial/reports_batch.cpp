// Source-authored report and real process-pool regression. Pending V08.
#include "support/migration_test.hpp"
#include "support/temporary_directory.hpp"
#include "migration/recording_fixture.hpp"
#include "fixture_paths.hpp"
#include "viz/spatial.hpp"
#include "recording/report.hpp"
#include "recording/spatial_analysis.hpp"
#include "recording/value_util.hpp"
#include "batch/plan.hpp"
#include "cli/arguments.hpp"
#include "platform/provenance.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {
void reports() {
    namespace viz=bike::viz;
    bt::invalid([]{(void)viz::spatial_attitude_plot({});});
    viz::Channels channels{{"time_s",std::vector<double>{0.,1.,2.,3.,4.}},
        {"spatial.attitude_valid",std::vector<double>{1.,1.,0.,1.,1.}},
        {"spatial.roll_rad",std::vector<double>{0.,.1,0.,.2,.3}},
        {"spatial.steer_angle_rad",std::vector<double>{0.,0.,0.,0.,0.}}};
    const auto report=viz::spatial_attitude_plot(channels);const auto& panel=report.panels.at(0);
    bt::require(panel.series.size()==3&&panel.series[0].x==std::vector<double>{0.,1.}&&panel.series[1].x==std::vector<double>{3.,4.},"invalid roll slot splits the line into two retained segments");
    bt::require(panel.markers.size()==1&&panel.markers[0].label.find("N/A")!=std::string::npos&&panel.series[2].x.size()==5,"gap is labelled N/A while steering remains independently available");
    auto bad=channels;std::get<std::vector<double>>(bad[2].second).pop_back();bt::invalid([&]{(void)viz::spatial_attitude_plot(bad);});
    bad=channels;std::get<std::vector<double>>(bad[1].second)[2]=2.;bt::invalid([&]{(void)viz::spatial_attitude_plot(bad);});
    const viz::Channels energy{{"time_s",std::vector<double>{0.,1.}},
        {"spatial.energy_time_s",std::vector<double>{1.,2.}},
        {"spatial.steer_work_j",std::vector<double>{1.,2.}},
        {"spatial.tyre_loss_j",std::vector<double>{3.,4.}},
        {"spatial.hold_work_j",std::vector<double>{5.,5.}}};
    const auto work=viz::spatial_energy_plot(energy);
    bt::require(work.panels[0].series.size()==3&&work.panels[0].series[0].x==std::vector<double>{1.,2.},"cumulative steering/tyre/hold work uses the accepted interval-end clock");
    const bt::TemporaryDirectory tmp("spatial-reports");
    const auto fixture=recording_fixture::publish(tmp.path()/"planar");
    const bike::recording::Recording planar(fixture.directory);
    bt::require(planar.schema_version()==1&&bike::recording::render_summary(planar)==bt::text(fixture.directory/"summary.md"),"planar source fixture retains byte-exact publication/report output");
    bt::invalid([&]{(void)bike::recording::spatial_report_channels(planar,0);});
    bt::invalid([&]{(void)bike::recording::window(planar,0,0.,1.,std::string("spatial.roll_rad"));});
    const auto historical_path=std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"tests/fixtures/migration/recording/good";
    bt::require(std::filesystem::is_regular_file(historical_path/"run.duckdb"),"V08 requires the existing acquired schema1 recording; missing binary is not a skipped comparison");
    const bike::recording::Recording historical(historical_path);
    bt::require(bike::recording::render_summary(historical)==bt::text(historical_path/"summary.md"),"existing schema1 summary remains byte-identical");
}
void processes() {
    namespace batch=bike::batch;namespace io=bike::io;using namespace bike::recording::value;
    const bt::TemporaryDirectory tmp("spatial-batch");
    const auto child=bike::platform::executable_path().string();
    std::vector<batch::Job> jobs(3);
    const std::array names{"slow","fast","crash"};
    const auto terrain=(std::filesystem::path(BIKE_SPATIAL_SOURCE_ROOT)/"examples/spatial/flat.toml").string();
    for(std::size_t i=0;i<jobs.size();++i) {
        auto& job=jobs[i];job.id=names[i];job.index=i;
        const auto seed=std::to_string(i);
        const std::array<std::string_view,10> args{"--spatial","3d","--terrain3d",terrain,"--duration","2","--seed",seed,"--rider","lumped"};
        batch::resolve_job_identity(job,bike::cli::parse_arguments("bike-research",args));
        bt::require(job.key&&job.key->size()==64,"job identity includes resolved spatial inputs");
        job.argv={child,"--child",names[i]};job.params=io::Value{io::Value::Object{{"duration",real(2.)}}};
    }
    batch::ExecuteOptions options;options.cwd=tmp.path();
    std::string expected;
    for(const unsigned workers:{1U,2U,4U}) {
        const auto results=batch::execute(jobs,workers,options);
        bt::require(results[0].exit_code==0&&results[1].exit_code==0&&results[2].exit_code==128+SIGTERM,"crashed process is classified while other jobs finish");
        const auto summary=batch::summarize(jobs,results,1.,"results.jsonl",false);
        const auto canonical=io::canonical_json(require(summary,"spatial_jobs"));
        if(expected.empty())expected=canonical;
        bt::require(canonical==expected,"workers 1/2/4 preserve canonical job-key/outcome summary despite completion order");
        bt::require(io::as_double(require(summary,"requested_sim_seconds"),"seconds")==4.,"crashed job is excluded from speed-stat duration");
    }
}
}
int main(int argc,char** argv) {
    if(argc==3&&std::string_view(argv[1])=="--child") {
        if(std::string_view(argv[2])=="crash"){std::raise(SIGTERM);return 99;}
        if(std::string_view(argv[2])=="slow")std::this_thread::sleep_for(std::chrono::milliseconds(30));
        std::cout<<"{\"outcome\":\"requested_completion\"}\n";return 0;
    }
    return bt::run([]{reports();processes();});
}
