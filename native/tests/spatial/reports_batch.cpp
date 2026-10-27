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
#include "platform/argv.hpp"
#include "platform/provenance.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

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
    auto isolated=channels;
    std::get<std::vector<double>>(isolated[1].second)={0.,1.,0.,1.,0.};
    const auto islands=viz::spatial_attitude_plot(isolated);
    bt::require(islands.panels[0].series[0].style==viz::SeriesStyle::points&&
        islands.panels[0].series[0].x==std::vector<double>{1.}&&
        islands.panels[0].series[1].style==viz::SeriesStyle::points&&
        islands.panels[0].series[1].x==std::vector<double>{3.},
        "single accepted samples between gaps remain visible points in interactive reports");
    bt::require(viz::to_html(islands).find("\"style\":\"points\"")!=std::string::npos,
        "offline HTML retains point rendering instead of an invisible one-vertex line");
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
void unavailable_time_extent() {
    namespace viz=bike::viz;
    viz::Channels channels{{"observations.time_s",std::vector<double>{0.,1.,2.,3.}},
        {"observations.valid",std::vector<double>{1.,1.,0.,0.}},
        {"observations.valid_mask",std::vector<double>{2047.,2047.,0.,0.}}};
    for(const auto field:{"observations.specific_force_sensor_mps2.","observations.angular_rate_sensor_rad_s."})
        for(int axis=0;axis<3;++axis)channels.emplace_back(std::string(field)+std::to_string(axis),std::vector<double>(4,0.));
    const auto trailing=viz::spatial_imu_plot(channels);
    for(const auto& panel:trailing.panels) {
        bt::require(panel.x_range==std::optional(std::pair{0.,3.}),"trailing invalid observations remain in the displayed time extent");
        bt::require(panel.series.at(0).x==std::vector<double>{0.,1.}&&panel.markers.at(0).first==2.,
            "the valid segment ends before the labelled dropout gap");
    }
    bt::require(viz::to_svg(trailing).find("N/A: X")!=std::string::npos,
        "native report actually draws the trailing-gap label inside its time axis");
    std::get<std::vector<double>>(channels[1].second).assign(4,0.);
    std::get<std::vector<double>>(channels[2].second).assign(4,0.);
    const auto unavailable=viz::spatial_imu_plot(channels);
    for(const auto& panel:unavailable.panels)
        bt::require(panel.series.empty()&&panel.x_range==std::optional(std::pair{0.,3.})&&!panel.markers.empty(),
            "all-invalid IMU evidence retains its actual clock and N/A marker without invented samples");
}
void fragmented_channels() {
    namespace viz=bike::viz;
    // Contact validity can toggle on every other sample over rough terrain.
    // Per-range series would exceed the plot's series bound and legend space;
    // the report must stay renderable and keep every accepted sample.
    constexpr std::size_t count=81;
    std::vector<double> stamps(count),valid(count),roll(count),steer(count,0.);
    for(std::size_t i=0;i<count;++i){stamps[i]=static_cast<double>(i);valid[i]=i%2==0?1.:0.;roll[i]=static_cast<double>(i)*.01;}
    const viz::Channels channels{{"time_s",stamps},{"spatial.attitude_valid",valid},
        {"spatial.roll_rad",roll},{"spatial.steer_angle_rad",steer}};
    const auto report=viz::spatial_attitude_plot(channels);
    const auto& panel=report.panels.at(0);
    std::size_t retained=0;
    for(const auto& series:panel.series)if(series.name=="roll")retained+=series.x.size();
    bt::require(panel.series.size()<=3&&retained==41,"fragmented channel keeps every accepted sample in a bounded series count");
    bt::require(panel.markers.size()<=16&&!panel.notes.empty(),"excess unavailable intervals are summarized instead of marked one by one");
    bt::require(!viz::to_svg(report).empty()&&!viz::to_html(report).empty(),"fragmented report remains renderable");
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
        bt::require(io::as_int64(require(job.identity,"threads"),"threads")==1,"default job identity records one resolved tyre worker");
        auto threaded_args=bike::cli::parse_arguments("bike-research",args);
        threaded_args.set("threads","2");
        batch::Job threaded;
        batch::resolve_job_identity(threaded,threaded_args);
        bt::require(io::as_int64(require(threaded.identity,"threads"),"threads")==2&&threaded.key!=job.key,
            "job identity records the requested tyre count independently of process-pool workers");
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
    // Raw argv subscripting is rejected by -Wunsafe-buffer-usage; the shared
    // wrapper performs the one ABI-to-span conversion.
    const auto arguments=bike::platform::main_arguments(argc,argv);
    if(arguments.size()==3&&std::string_view(arguments[1])=="--child") {
        const std::string_view mode(arguments[2]);
        if(mode=="crash"){std::raise(SIGTERM);return 99;}
        if(mode=="slow")std::this_thread::sleep_for(std::chrono::milliseconds(30));
        std::cout<<"{\"outcome\":\"requested_completion\"}\n";return 0;
    }
    return bt::run([]{reports();unavailable_time_extent();fragmented_channels();processes();});
}
