#include "application.hpp"
#include "physical_capture.hpp"
#include "viewer_options.hpp"
#include "../app/prepare.hpp"
#include "../initialization/sag_fit.hpp"
#include "../io/json.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../platform/signals.hpp"
#include "../recording/capture.hpp"
#include "../recording/publish.hpp"
#include "../research/preview_csv.hpp"
#include "../runtime/run_limits.hpp"
#include "../runtime/setup_values.hpp"
#include "../terrain/trackfile.hpp"
#include "../viewer/session.hpp"
#include "../viz/plot.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>

namespace bike::cli {
namespace {
using namespace runtime::setupvalue;
viz::Channels plot_columns(const runtime::SampleColumns& columns) {
    viz::Channels out;for(const auto& [name,values]:columns)out.emplace_back(name,values);return out;
}
runtime::WireObject preview_row(const runtime::RenderFrame& frame,const terrain::TrackSpec& track,
    std::optional<double> achieved_rtf) {
    auto row=runtime::sample_wire::object(runtime::sample_wire::required(frame.state.view,"preview_row"));
    runtime::sample_wire::set(row,"rtf",achieved_rtf?runtime::Wire{*achieved_rtf}:runtime::Wire{nullptr});
    const double position=frame.presentation.position_m;
    runtime::sample_wire::set(row,"grade_pct",track.grade_profile?track.grade_profile->slope(position)*100.:0.);
    const auto markers=track.markers();
    if(!markers.empty()) {
        const auto nearest=std::ranges::min_element(markers,[&](const auto& a,const auto& b){return std::abs(a.first-position)<std::abs(b.first-position);});
        runtime::sample_wire::set(row,"obstacle",nearest->second);
        runtime::sample_wire::set(row,"obstacle_distance_m",nearest->first-position);
    }
    for(const auto key:{"model_valid","numerically_valid"}) {
        const auto* value=runtime::sample_wire::find(frame.state.model_status,key);
        const auto* valid=value?std::get_if<bool>(&value->value):nullptr;
        runtime::sample_wire::set(row,key,valid?runtime::Wire{static_cast<std::int64_t>(*valid)}:runtime::Wire{nullptr});
    }
    runtime::sample_wire::set(row,"termination_reason",frame.state.outcome?runtime::Wire{*frame.state.outcome}:runtime::Wire{std::string{}});
    return row;
}
[[gnu::noinline]] void apply_sag(initialization::SetupRequest& request, const Arguments& args) {
    const double pct=args.real("sag");if(pct<=0.||pct>=100.)throw ArgumentError("sag percentage must lie strictly between 0 and 100");
    const auto& bike=request.model.bike;
    const auto fitted=initialization::fit_sag(initialization::build_equilibrium_evaluator(request),
        {bike.fork_travel*pct/100.,bike.rear_wheel_travel*pct/100.},
        {bike.fork_initial_psi,bike.shock_stiffness},{10.,1000.},{400.,250000.});
    request.model.bike.fork_initial_psi=fitted[0];request.model.bike.shock_stiffness=fitted[1];
}
// ResolvedRide, PreparedSimulation and the by-value arguments of prepare and
// make_ride are each several KB; heap ownership keeps this orchestration
// frame inside the strict bound.
int physical_run(std::unique_ptr<ResolvedRide> resolved) {
    const auto& args=resolved->arguments;
    auto& request=resolved->setup;
    if(args.has("sag")) apply_sag(request,args);
    platform::InterruptShield interrupt;
    auto prepared=app::prepare(std::make_unique<initialization::SetupRequest>(request));
    const auto metadata=launch_metadata(request,*prepared,args.optional_text("seed"));
    std::optional<recording::Catalog> catalog;
    std::vector<recording::Asset> assets;
    if(args.flag("record")) {catalog=model_catalog(prepared->bootstrap.model);assets=physical_assets(request,*prepared,metadata);}
    auto runtime=app::make_ride(std::move(prepared));
    const auto dt=request.model.physics.value().timestep_s; // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
    const auto limits=runtime::RunLimits::for_track(request.track.length_m,request.start_x_m,dt);
    const double requested_steps=args.has("duration")?std::ceil(args.real("duration")/dt):static_cast<double>(limits.max_steps);
    if(!std::isfinite(requested_steps)||requested_steps>=static_cast<double>(std::numeric_limits<std::int64_t>::max()))throw ArgumentError("duration exceeds step range");
    const auto max_steps=static_cast<std::int64_t>(requested_steps);
    runtime->set_finish_position(limits.finish_x_m);
    recording::RunCapture capture(recording::make_run_metadata("native",args.flag("headless")?"physical_headless":"physical_viewer",metadata),args.flag("record"));
    std::int64_t episode_id=0;
    auto journal=std::make_unique<recording::EpisodeJournal>(capture,episode_id);
    bool sealed=false;std::string reason;
    io::Value summary;
    std::optional<recording::Publication> publication;
    // The event journal uses the launch clock across every episode. Resetting
    // an episode rebases its watchdog only, never manifest/event timestamps.
    const double launch_started=platform::monotonic_seconds();
    double episode_started=launch_started;
    const auto wall=[&]{return platform::monotonic_seconds()-launch_started;};
    std::ostringstream preview_bytes;
    std::unique_ptr<research::PhysicalPreviewCsv> preview;
    if(capture.enabled()&&!args.flag("headless"))preview=std::make_unique<research::PhysicalPreviewCsv>(preview_bytes);
    const auto terminal=[&] {
        const auto state=runtime->snapshot();
        if(interrupt.deferred()){reason="operator_stop";return true;}
        if(state.outcome){reason=*state.outcome;return true;}
        if(state.step>=max_steps){reason=args.has("duration")?"duration_reached":"step_cap";return true;}
        if(platform::monotonic_seconds()-episode_started>=limits.max_wall_clock_s){reason="wall_clock_cap";return true;}
        return false;
    };
    const auto seal=[&](std::string_view end_reason) {
        if(sealed)return;
        if(capture.enabled()) {
            auto owned=runtime->seal_episode();
            auto value=physical_summary(*runtime,metadata,end_reason);
            set(value,"recording_retention",text("full"));set(value,"measurement_decimation",integer(request.runtime.measurement_decimation));
            const auto events=recording::derived_events(owned,episode_id,journal->next_id(),wall());
            auto evidence=std::make_shared<recording::EpisodeEvidence>();
            evidence->episode_id=episode_id;evidence->generation=owned.generation;
            evidence->physical=recording::make_physical_source(std::move(owned),*catalog,true);
            evidence->summary=value;evidence->assets=assets;
            capture.add_episode(std::move(evidence));journal->emit_all(events);summary=std::move(value);
        } else {runtime->flush();summary=physical_summary(*runtime,metadata,end_reason);}
        sealed=true;
    };
    std::exception_ptr failure;
    try {
        (void)journal->emit("episode_started",wall(),0,{},Value{Object{{"record",text("lifecycle")}}});
        if(args.flag("headless")) {
            while(!terminal()) {
                const auto step=runtime->snapshot().step;
                (void)runtime->advance(std::min(max_steps,step+4000),{},0.,0.,.02);
                runtime->release_samples();
                if(const auto warning=runtime->take_pending_warning())std::cerr<<"WARNING: "<<*warning<<'\n';
            }
        } else {
            double front=0.,rear=0.;
            viewer::SessionHooks hooks;
            hooks.close_requested=[&]{if(!interrupt.deferred())return false;reason="operator_stop";return true;};
            hooks.frame=[&]{return runtime->render_frame();};hooks.terminal=terminal;
            hooks.presented=[&](const runtime::RenderFrame& frame,std::optional<double> achieved){
                if(preview&&preview->due(frame.state.time_s,frame.state.generation))
                    preview->write(frame.state.time_s,frame.state.generation,preview_row(frame,request.track,achieved));
            };
            hooks.advance=[&](std::int64_t target,double budget){(void)runtime->advance(std::min(target,max_steps),{},front,rear,budget);runtime->release_samples();};
            hooks.brakes=[&](double f,double r){front=f;rear=r;(void)journal->emit("brakes",wall(),runtime->snapshot().step,{},Value{Object{{"front",real(f)},{"rear",real(r)}}});};
            hooks.pause=[&](bool paused){(void)journal->emit(paused?"pause":"resume",wall(),runtime->snapshot().step);};
            hooks.reset=[&]{seal("reset");runtime->reset();++episode_id;sealed=false;reason.clear();episode_started=platform::monotonic_seconds();journal=std::make_unique<recording::EpisodeJournal>(capture,episode_id);(void)journal->emit("episode_started",wall(),0);};
            hooks.stop=[&]{if(reason.empty())reason="viewer_closed";};
            const auto options=ride_viewer_options(args,dt);
            (void)viewer::run(std::move(hooks),runtime->model_image(),options);
            if(reason.empty())reason="viewer_closed";
        }
    } catch(...) {failure=std::current_exception();reason="simulation_error";}
    // A strict trailing flush can report an already committed rejection once.
    // Retry only the seal over that prefix; no physics advance is retried.
    try {seal(reason);}catch(...){if(!failure)failure=std::current_exception();try{seal(reason);}catch(const std::exception& error){std::cerr<<"recording seal failed: "<<error.what()<<'\n';}}
    if(preview) {
        try {
            const auto frame=runtime->render_frame();
            preview->write(frame.state.time_s,frame.state.generation,preview_row(frame,request.track,{}));
            preview->write_marker("run ended; "+reason+"; t="+geometry::detail::py_fixed(frame.state.time_s,6)+"s");
            preview->close();
        } catch(...) {if(!failure)failure=std::current_exception();else std::cerr<<"preview finalization also failed\n";}
    }
    if(capture.enabled()&&sealed) {
        try {publication=recording::finalize(capture,args.text("out","output/ride"));}
        catch(...) {if(!failure)failure=std::current_exception();else std::cerr<<"recording finalization also failed\n";}
    }
    if(publication) {
        try {
            if(preview){const auto bytes=preview_bytes.str();platform::write_exclusive(publication->directory/"preview.csv",std::as_bytes(std::span(bytes)));}
            if(!args.flag("no_plots")) {
                const auto columns=plot_columns(runtime->recorded_columns());
                if(!columns.empty())for(const auto& report:viz::physical_ride_plots(columns))viz::write_artifact(report,publication->directory/(report.title+".png"));
                if(preview)(void)viz::plot_physical_ride_html(viz::load_ride_csv(publication->directory/"preview.csv"),publication->directory);
            }
        } catch(...) {if(!failure)failure=std::current_exception();else std::cerr<<"plot/preview publication also failed\n";}
    }
    runtime->close();
    if(failure)std::rethrow_exception(failure);
    std::cout<<io::recording_json(summary)<<'\n';
    if(publication)std::cout<<"[bike-ride] recording: "<<publication->directory<<'\n';
    if(reason=="operator_stop")return 130;
    if(const auto* status=io::find_member(io::as_object(summary,"summary"),"model_status"))
        if(const auto* valid=io::find_member(io::as_object(*status,"model_status"),"model_valid");valid&&!io::as_bool(*valid,"model_valid"))return 2;
    return reason=="duration_reached"||reason=="end_of_track"||reason=="viewer_closed"?0:1;
}

} // namespace
int ride_command(const Arguments& input) {
    if(input.flag("list_tracks")){for(const auto& name:terrain::available_presets())std::cout<<name<<'\n';return 0;}
    if(input.has("dump_track")){std::cout<<terrain::dump_track(terrain::get_preset(input.text("dump_track")));return 0;}
    if(input.has("html")){const auto root=std::filesystem::path(input.text("html"));std::cout<<viz::plot_physical_ride_html(viz::load_ride_csv(root/"preview.csv"),root)<<'\n';return 0;}
    // ResolvedRide and ResearchRequest are each several KB — heap ownership
    // keeps this dispatcher frame inside the strict bound.
    auto resolved=resolve_ride(input);
    if(input.flag("preview")){const auto path=std::filesystem::path(input.text("out"))/(resolved->setup.track.name+"_profile.png");std::filesystem::create_directories(path.parent_path());viz::write_artifact(viz::track_profile(resolved->setup.track),path);std::cout<<path<<'\n';return 0;}
    if(input.flag("research")) {
        auto request=std::make_unique<research::ResearchRequest>();request->setup=resolved->setup;
        request->experiment.duration_s=input.real("duration",90.);request->experiment.seed_decimal=resolved->setup.seed_decimal;
        request->experiment.control_period_s=input.real("control_period",.01);request->experiment.actuator_delay_s=input.real("actuator_delay",.005);request->experiment.record_decimation=resolved->setup.runtime.measurement_decimation;request->experiment.stop_on_model_violation=resolved->setup.runtime.strict;request->experiment.async_telemetry=input.flag("async_telemetry");
        request->sensors.sample_period_s=input.real("sensor_period",.005);request->sensors.delivery.latency_s=input.real("sensor_delay",.01);
        request->road_resolution_m=input.real("road_resolution",.005);
        request->programs=research::load_programs(input.has("rider_program")?std::optional<std::filesystem::path>(input.text("rider_program")):std::nullopt,{});
        if(input.has("demand"))request->programs.demand_frames=research::constant_demand(input.real("demand")).demand_frames;
        request->capture_mode=input.flag("record")?runtime::ResearchCaptureMode::full:runtime::ResearchCaptureMode::off;
        auto args=resolved->arguments;args.set("assist",input.has("demand")?"false":"true");
        return run_research(std::move(request),args,!input.flag("headless"));
    }
    return physical_run(std::move(resolved));
}
} // namespace bike::cli
