#include "application.hpp"
#include "physical_capture.hpp"
#include "viewer_options.hpp"
#include "../app/prepare.hpp"
#include "../configuration/load.hpp"
#include "../initialization/sag_fit.hpp"
#include "../initialization/source_json.hpp"
#include "../io/hash.hpp"
#include "../io/json.hpp"
#include "../io/npy.hpp"
#include "../legacy/metrics.hpp"
#include "../legacy/ride.hpp"
#include "../model/suspension_setup.hpp"
#include "../platform/files.hpp"
#include "../platform/provenance.hpp"
#include "../platform/signals.hpp"
#include "../recording/capture.hpp"
#include "../recording/publish.hpp"
#include "../recording/spatial_analysis.hpp"
#include "../research/preview_csv.hpp"
#include "../runtime/setup_values.hpp"
#include "../spatial/bootstrap_state.hpp"
#include "../terrain/trackfile.hpp"
#include "../terrain/source3d.hpp"
#include "../viewer/session.hpp"
#include "../viz/plot.hpp"
#include "../viz/spatial.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace bike::cli {
namespace {
using namespace runtime::setupvalue;
viz::Channels plot_columns(const runtime::SampleColumns& columns) {
    viz::Channels out;for(const auto& [name,values]:columns)out.emplace_back(name,values);return out;
}
io::Value drivetrain_value(const legacy::DrivetrainSpecs& d) {
    Array cassette;for(const int teeth:d.cassette)cassette.push_back(integer(teeth));
    return Value{Object{{"chainring_teeth",integer(d.chainring_teeth)},{"cog_teeth",integer(d.cog_teeth)},
        {"cassette",Value{std::move(cassette)}},{"auto_shift",Value{d.auto_shift}},
        {"target_cadence_min_rpm",real(d.target_cadence_min_rpm)},{"target_cadence_max_rpm",real(d.target_cadence_max_rpm)},
        {"shift_cooldown_s",real(d.shift_cooldown_s)},{"shift_cut_duration_s",real(d.shift_cut_duration_s)},
        {"ripple_depth",real(d.ripple_depth)},{"rider_torque_ceiling_nm",real(d.rider_torque_ceiling_nm)},
        {"rider_power_ceiling_w",real(d.rider_power_ceiling_w)},{"assist_torque_ceiling_nm",real(d.assist_torque_ceiling_nm)},
        {"assist_power_ceiling_w",real(d.assist_power_ceiling_w)},{"assist_response_s",real(d.assist_response_s)},
        {"cutoff_speed_kmh",real(d.cutoff_speed_kmh)},{"cutoff_taper_kmh",real(d.cutoff_taper_kmh)},
        {"crank_phase_deg",real(d.crank_phase_deg)},{"crank_hold_stiffness_nm_rad",real(d.crank_hold_stiffness_nm_rad)},
        {"crank_hold_damping_nms_rad",real(d.crank_hold_damping_nms_rad)}}};
}
std::optional<std::string> ride_track_seed(const Arguments& args) {
    if(args.has("seed"))return args.text("seed");
    const auto name=args.text("track","road_worn");
    if(name=="road_smooth")return terrain::road_smooth_spec().seed;
    if(name=="road_worn")return terrain::road_worn_spec().seed;
    if(name=="road_broken")return terrain::road_broken_spec().seed;
    if(name=="alpine_scree")return std::to_string(terrain::ALPINE_PRESET_SEED);
    if(name=="alpine_rocky")return std::to_string(terrain::ROCKY_PRESET_SEED);
    const auto names=terrain::available_presets();
    if(std::ranges::find(names,name)!=names.end())return {};
    const auto bytes=io::read_numpy_bytes(name);std::string source;source.reserve(bytes.size());
    for(const std::byte byte:bytes)source.push_back(static_cast<char>(byte));
    const auto document=terrain::parse_track_toml(source,name);
    const auto* generator=io::find_member(io::as_object(document,"track"),"generator");
    return generator?std::optional(terrain::generator_from_dict(*generator).seed):std::nullopt;
}
std::string legacy_directory_name(const legacy::RideOptions& options,const std::optional<std::string>& seed) {
    const auto& track=options.setup.track;
    auto name=track.name+"_"+geometry::detail::py_g(options.setup.runtime.speed_kph);
    if(seed)name+="_s"+*seed;
    if(options.tyre.pneumatic())name+="_pneumatic-"+options.tyre.tier+"_"+geometry::detail::py_fixed(options.tyre.front.pressure_bar,2)+"-"+
        geometry::detail::py_fixed(options.tyre.rear.pressure_bar,2)+"bar_"+options.tyre.surface.value_or(track.surface);
    const auto drivetrain=drivetrain_value(options.drivetrain);
    const auto physics=configuration::detail::plain_document(configuration::to_value(options.setup.model.physics.value())); // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
    const bool standard=options.drive_mode=="motor"&&!options.visual_pedalling&&options.legs.value_or("rigid")=="rigid"&&
        io::canonical_json(drivetrain)==io::canonical_json(drivetrain_value(legacy::DrivetrainSpecs{}))&&
        io::canonical_json(physics)==io::canonical_json(configuration::detail::plain_document(configuration::to_value(configuration::default_physics())));
    if(!standard){
        const auto settings=Value{Object{{"mode",text(options.drive_mode)},{"assist",text(options.drive_mode=="pedelec"?options.assist_mode:"off")},
            {"drivetrain",drivetrain},{"legs",text(options.legs.value_or(options.drive_mode!="motor"||options.visual_pedalling?"articulated":"rigid"))},
            {"visual_pedalling",Value{options.visual_pedalling}},{"physics",physics}}};
        const auto encoded=initialization::detail::source_json(settings);
        name+="_"+options.drive_mode+"_"+io::sha256(std::as_bytes(std::span(encoded))).substr(0,16);
    }
    return name;
}
} // namespace
runtime::WireObject physical_preview_row(const runtime::RenderFrame& frame,const terrain::TrackSpec& track,
    std::optional<double> achieved_rtf) {
    auto row=runtime::sample_wire::object(runtime::sample_wire::required(frame.state.view,"preview_row"));
    runtime::sample_wire::set(row,"rtf",achieved_rtf?runtime::Wire{*achieved_rtf}:runtime::Wire{nullptr});
    const double position=frame.presentation.position_m;
    // A spatial SetupRequest carries a planar placeholder, not the authored
    // Terrain3D. No accepted directional grade is present in this CSV input.
    runtime::sample_wire::set(row,"grade_pct",frame.presentation.spatial ? runtime::Wire{nullptr} :
        runtime::Wire{track.grade_profile?track.grade_profile->slope(position)*100.:0.});
    const auto markers=track.markers();
    if(!frame.presentation.spatial&&!markers.empty()) {
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
int physical_ride_exit_code(std::string_view reason,const io::Value& summary) {
    if(reason=="operator_stop")return 130;
    if(const auto* status=io::find_member(io::as_object(summary,"summary"),"model_status")) {
        const auto& fields=io::as_object(*status,"model_status");
        for(const auto key:{"model_valid","numerically_valid"}) {
            const auto* value=io::find_member(fields,key);
            const auto* valid=value?std::get_if<bool>(&value->data):nullptr;
            if(valid&&!*valid)return 2;
        }
    }
    return reason=="duration_reached"||reason=="end_of_track"||reason=="viewer_closed"||
        reason==spatial::to_string(spatial::SpatialReason::requested_completion)?0:1;
}
namespace {
[[gnu::noinline]] void apply_sag(initialization::SetupRequest& request, const Arguments& args) {
    const double pct=args.real("sag");if(pct<=0.||pct>=100.)throw ArgumentError("sag percentage must lie strictly between 0 and 100");
    const auto& bike=request.model.bike;
    const auto fitted=initialization::fit_sag(initialization::build_equilibrium_evaluator(request),
        {bike.fork_travel*pct/100.,bike.rear_wheel_travel*pct/100.},
        {bike.fork_initial_psi,bike.shock_stiffness},{10.,1000.},{400.,250000.});
    request.model.bike.fork_initial_psi=fitted[0];request.model.bike.shock_stiffness=fitted[1];
}

double finalization_elapsed(double started) {
    const double elapsed = platform::monotonic_seconds() - started;
    if (!std::isfinite(elapsed) || elapsed < 0.)
        throw std::invalid_argument("viewer finalization clock must be finite and monotone");
    return elapsed;
}
} // namespace
[[gnu::noinline]] io::Value viewer_performance_value(const viewer::PacingReport& pacing, const io::Value& summary,
    const ViewerFinalizationTimes& finalization, bool fast_profile, bool strict, bool recording_enabled) {
    const auto optional_real = [](std::optional<double> value) { return value ? real(*value) : Value{}; };
    const auto counter = [](std::uint64_t value) { return Value{io::Number{std::to_string(value)}}; };
    const auto& status = field(summary, "model_status");
    const auto accepted_true=[](const Value& value) {
        const auto* flag=std::get_if<bool>(&value.data);return flag&&*flag;
    };
    const bool physical_valid = accepted_true(field(status, "model_valid")) &&
        accepted_true(field(status, "numerically_valid")) &&
        io::is_null(field(summary, "first_failure"));
    const auto& reason = io::as_string(field(field(summary, "outcome"), "reason"), "outcome.reason");
    const bool complete = reason == "duration_reached" ||
        reason == spatial::to_string(spatial::SpatialReason::requested_completion);
    const double finalization_s = finalization.seal_s + finalization.preview_s +
        finalization.publication_s + finalization.close_s;
    const double total_s = pacing.wall_s + finalization_s;
    if (!std::isfinite(total_s) || total_s < 0.)
        throw std::invalid_argument("viewer total measurement time must be finite and nonnegative");
    std::optional<double> total_rtf;
    if (total_s > 0.) {
        const double factor=pacing.simulated_s/total_s;
        if(std::isfinite(factor))total_rtf=factor;
    }
    // The accumulated integration clock may lie a few ulps below 30 seconds.
    // Apply the report protocol's isclose tolerance only to window length;
    // neither achieved RTF nor its 1x threshold receives favorable rounding.
    const bool full_window = pacing.simulated_s >= 30. ||
        std::abs(pacing.simulated_s - 30.) <= 1e-9 * std::max(30., std::abs(pacing.simulated_s));
    const bool eligible = pacing.one_x_eligible && complete && physical_valid && fast_profile && strict &&
        full_window && total_rtf && std::isfinite(*total_rtf);
    Array reasons;
    if (!pacing.stopped || !pacing.terminal || !complete) reasons.push_back(text("incomplete_requested_episode"));
    if (pacing.wall_s <= 0. || pacing.simulated_s <= 0.) reasons.push_back(text("empty_timing_window"));
    if (!pacing.mean_achieved_rtf) reasons.push_back(text("invalid_achieved_rate"));
    if (pacing.presented_frames == 0) reasons.push_back(text("no_presented_frames"));
    if (pacing.hidden) reasons.push_back(text("hidden_window"));
    if (pacing.framebuffer_unavailable) reasons.push_back(text("framebuffer_unavailable"));
    if (pacing.paused) reasons.push_back(text("pause_in_measurement_window"));
    if (pacing.scaled) reasons.push_back(text("time_scale_not_continuously_one"));
    if (pacing.resets != 0) reasons.push_back(text("multiple_generations"));
    if (!physical_valid) reasons.push_back(text("physical_or_numerical_invalidity"));
    if (!full_window) reasons.push_back(text("less_than_30_simulated_seconds"));
    if (!fast_profile) reasons.push_back(text("not_free_root_fast_tyre_profile"));
    if (!strict) reasons.push_back(text("non_strict_run"));
    return Value{Object{
        {"viewer_performance_schema", integer(1)},
        {"measurement_scope", text("first viewer window through terminal presentation/callbacks; setup and terminal idle excluded")},
        {"simulation_s", real(pacing.simulated_s)}, {"viewer_wall_s", real(pacing.wall_s)},
        {"mean_achieved_rtf", optional_real(pacing.mean_achieved_rtf)},
        {"render_wall_s", real(pacing.render_wall_s)}, {"presented_frames", counter(pacing.presented_frames)},
        {"frames_per_s", optional_real(pacing.frames_per_s)},
        {"frame_count_scope", text("positive framebuffer draw followed by successful swap only")},
        {"render_interval_s", real(pacing.render_interval_s)},
        {"first_frame_delay_s", pacing.presented_frames ? real(pacing.first_frame_delay_s) : Value{}},
        {"frame_lateness", Value{Object{
            {"units", text("seconds per completed presentation interval, never per physics step")},
            {"definition", text("max(0, actual presentation interval - requested render interval); first frame excluded")},
            {"late_frames", counter(pacing.late_frames)},
            {"total_s", real(pacing.total_frame_lateness_s)},
            {"maximum_s", real(pacing.max_frame_lateness_s)},
            {"mean_s", optional_real(pacing.mean_frame_lateness_s)}}}},
        {"debt_s", real(pacing.debt_s)}, {"maximum_debt_s", real(pacing.max_debt_s)},
        {"initial_time_scale", integer(pacing.initial_scale)},
        {"paused", Value{pacing.paused}}, {"scaled", Value{pacing.scaled}},
        {"hidden", Value{pacing.hidden}}, {"framebuffer_unavailable", Value{pacing.framebuffer_unavailable}},
        {"resets", counter(pacing.resets)}, {"ended_at_terminal_frame", Value{pacing.terminal}},
        {"strict", Value{strict}}, {"recording_enabled", Value{recording_enabled}},
        {"finalization_wall_s", Value{Object{{"seal_and_flush", real(finalization.seal_s)},
            {"preview", real(finalization.preview_s)}, {"recording_publication", real(finalization.publication_s)},
            {"runtime_close", real(finalization.close_s)}}}},
        {"viewer_and_finalization_wall_s", real(total_s)},
        {"mean_rtf_with_finalization", optional_real(total_rtf)},
        {"plot_export_wall_s", real(finalization.export_s)},
        {"finalization_scope", text("seal/flush, preview, recording publication and runtime close included; setup, terminal idle, window teardown and plot export excluded")},
        {"eligible_1x_measurement", Value{eligible}}, {"ineligibility_reasons", Value{std::move(reasons)}},
        {"target_rtf", real(1.)}, {"target_met", eligible ? Value{*total_rtf >= 1.} : Value{}},
        {"full_acceptance_verified", Value{false}},
        {"pending_gates", Value{Array{text("V10 matching final candidate and both-platform evidence"),
            text("V10 real-display visual acceptance and complete physical qualification")}}}
    }};
}
namespace {
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
    const double physical_end = prepared->bootstrap.config.terrain3d
        ? prepared->bootstrap.config.terrain3d->x_max_m() : request.track.length_m;
    auto runtime=app::make_ride(std::move(prepared));
    const auto dt=request.model.physics.value().timestep_s; // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
    const auto limits=legacy::RunLimits::for_track(physical_end,request.start_x_m,dt);
    const double requested_steps=args.has("duration")?std::ceil(args.real("duration")/dt):static_cast<double>(limits.max_steps);
    if(!std::isfinite(requested_steps)||requested_steps>=static_cast<double>(std::numeric_limits<std::int64_t>::max()))throw ArgumentError("duration exceeds step range");
    const auto max_steps=static_cast<std::int64_t>(requested_steps);
    // Spatial runs use the committed W33 terrain-domain/corridor outcome.
    // A planar TrackSpec is not their physical finish boundary.
    if (request.model.physics->spatial) {
        if(args.has("duration"))runtime->set_completion_step(max_steps);
    } else runtime->set_finish_position(limits.finish_x_m);
    recording::RunCapture capture(recording::make_run_metadata("native",args.flag("headless")?"physical_headless":"physical_viewer",metadata),args.flag("record"));
    std::int64_t episode_id=0;
    auto journal=std::make_unique<recording::EpisodeJournal>(capture,episode_id);
    bool sealed=false;std::string reason;
    io::Value summary;
    std::optional<recording::Publication> publication;
    std::optional<viewer::PacingReport> viewer_pacing;
    ViewerFinalizationTimes viewer_finalization;
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
            evidence->summary=value;evidence->assets=physical_final_assets(assets,*runtime,metadata);
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
                    preview->write(frame.state.time_s,frame.state.generation,physical_preview_row(frame,request.track,achieved));
            };
            hooks.advance=[&](std::int64_t target,double budget){(void)runtime->advance(std::min(target,max_steps),{},front,rear,budget);runtime->release_samples();};
            hooks.brakes=[&](double f,double r){front=f;rear=r;(void)journal->emit("brakes",wall(),runtime->snapshot().step,{},Value{Object{{"front",real(f)},{"rear",real(r)}}});};
            hooks.pause=[&](bool paused){(void)journal->emit(paused?"pause":"resume",wall(),runtime->snapshot().step);};
            hooks.reset=[&]{seal("reset");runtime->reset();++episode_id;sealed=false;reason.clear();episode_started=platform::monotonic_seconds();journal=std::make_unique<recording::EpisodeJournal>(capture,episode_id);(void)journal->emit("episode_started",wall(),0);};
            hooks.stop=[&]{if(reason.empty())reason="viewer_closed";};
            const auto options=ride_viewer_options(args,dt);
            const auto viewer_result=viewer::run(std::move(hooks),runtime->model_image(),options);
            viewer_pacing=viewer_result.pacing;
            if(reason.empty())reason="viewer_closed";
        }
    } catch(...) {failure=std::current_exception();reason="simulation_error";}
    // A strict trailing flush can report an already committed rejection once.
    // Retry only the seal over that prefix; no physics advance is retried.
    const double seal_started=viewer_pacing?platform::monotonic_seconds():0.;
    try {seal(reason);}catch(...){if(!failure)failure=std::current_exception();try{seal(reason);}catch(const std::exception& error){std::cerr<<"recording seal failed: "<<error.what()<<'\n';}}
    if(viewer_pacing)viewer_finalization.seal_s=finalization_elapsed(seal_started);
    if(preview) {
        const double preview_started=viewer_pacing?platform::monotonic_seconds():0.;
        try {
            const auto frame=runtime->render_frame();
            preview->write(frame.state.time_s,frame.state.generation,physical_preview_row(frame,request.track,{}));
            preview->write_marker("run ended; "+reason+"; t="+geometry::detail::py_fixed(frame.state.time_s,6)+"s");
            preview->close();
        } catch(...) {if(!failure)failure=std::current_exception();else std::cerr<<"preview finalization also failed\n";}
        if(viewer_pacing)viewer_finalization.preview_s=finalization_elapsed(preview_started);
    }
    if(capture.enabled()&&sealed) {
        const double publication_started=viewer_pacing?platform::monotonic_seconds():0.;
        try {publication=recording::finalize(capture,args.text("out","output/ride"));}
        catch(...) {if(!failure)failure=std::current_exception();else std::cerr<<"recording finalization also failed\n";}
        if(viewer_pacing)viewer_finalization.publication_s=finalization_elapsed(publication_started);
    }
    if(publication) {
        const double export_started=viewer_pacing?platform::monotonic_seconds():0.;
        try {
            if(preview){const auto bytes=preview_bytes.str();platform::write_exclusive(publication->directory/"preview.csv",std::as_bytes(std::span(bytes)));}
            if(!args.flag("no_plots")) {
                if(request.model.spatial) {
                    const recording::Recording accepted(publication->directory);
                    const auto columns=recording::spatial_report_channels(accepted,episode_id);
                    std::vector<viz::Report> reports{viz::spatial_attitude_plot(columns),viz::spatial_contact_plot(columns),viz::spatial_energy_plot(columns)};
                    if(viz::numeric(columns,"spatial.front_wheel_x_m"))reports.push_back(viz::spatial_wheel_path_plot(columns));
                    if(const auto* times=viz::numeric(columns,"observations.time_s");times&&!times->empty())reports.push_back(viz::spatial_imu_plot(columns));
                    for(const auto& report:reports)viz::write_artifact(report,publication->directory/(report.title+".png"));
                    const auto html=viz::to_html(reports);
                    platform::write_exclusive(publication->directory/"spatial.html",std::as_bytes(std::span(html)));
                } else {
                    const auto columns=plot_columns(runtime->recorded_columns());
                    if(!columns.empty())for(const auto& report:viz::physical_ride_plots(columns))viz::write_artifact(report,publication->directory/(report.title+".png"));
                    if(preview)(void)viz::plot_physical_ride_html(viz::load_ride_csv(publication->directory/"preview.csv"),publication->directory);
                }
            }
        } catch(...) {if(!failure)failure=std::current_exception();else std::cerr<<"plot/preview publication also failed\n";}
        if(viewer_pacing)viewer_finalization.export_s=finalization_elapsed(export_started);
    }
    const double close_started=viewer_pacing?platform::monotonic_seconds():0.;
    runtime->close();
    if(viewer_pacing)viewer_finalization.close_s=finalization_elapsed(close_started);
    if(failure)std::rethrow_exception(failure);
    if(viewer_pacing&&request.model.physics&&request.model.physics->spatial) {
        const auto& spatial_config=*request.model.physics->spatial;
        const bool fast_profile=spatial_config.root==spatial::RootKind::free6&&spatial_config.tyre.tier==spatial::TyreTier::fast;
        set(summary,"viewer_performance",viewer_performance_value(*viewer_pacing,summary,
            viewer_finalization,fast_profile,request.runtime.strict,capture.enabled()));
    }
    std::cout<<io::recording_json(summary)<<'\n';
    if(publication)std::cout<<"[bike-ride] recording: "<<publication->directory<<'\n';
    return physical_ride_exit_code(reason,summary);
}

// RideOptions embeds the multi-KB setup request; the options builder and the
// headless tail own their own noinline frames so legacy_run stays small.
[[gnu::noinline]] std::unique_ptr<legacy::RideOptions> legacy_options(const ResolvedRide& resolved,
    const std::optional<std::string>& seed, io::Value& extras) {
    const auto& args=resolved.arguments;
    auto options=std::make_unique<legacy::RideOptions>();options->setup=resolved.setup;
    options->drive_mode=args.text("drive_mode","motor");options->assist_mode=args.text("assist","tour");
    options->visual_pedalling=args.flag("visual_pedalling");options->legs=args.optional_text("legs");
    if(options->visual_pedalling&&options->drive_mode!="motor")throw ArgumentError("--visual-pedalling applies only to --drive-mode motor");
    if(options->visual_pedalling&&options->legs==std::optional<std::string>("rigid"))throw ArgumentError("--visual-pedalling needs articulated legs");
    if(options->legs==std::optional<std::string>("articulated")&&options->drive_mode=="motor"&&!options->visual_pedalling)throw ArgumentError("articulated legs require pedal/pedelec or --visual-pedalling");
    if(!options->legs)options->legs=options->drive_mode!="motor"||options->visual_pedalling?"articulated":"rigid";
    options->setup.model.rider.value().legs=options->legs.value(); // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
    options->tyre.model=args.text("tyre_model","sphere");options->tyre.tier=args.text("tyre_tier","fast");options->tyre.surface=args.optional_text("surface");
    const auto pressure=args.text("tyre_pressure");const auto slash=pressure.find('/');
    if(slash==std::string::npos||pressure.find('/',slash+1)!=std::string::npos)throw ArgumentError("--tyre-pressure must be FRONT/REAR in bar");
    Arguments numbers;numbers.set("front",pressure.substr(0,slash));numbers.set("rear",pressure.substr(slash+1));
    const auto front=numbers.real("front"),rear=numbers.real("rear");
    if(front<legacy::PRESSURE_MIN_BAR||front>legacy::PRESSURE_MAX_BAR||rear<legacy::PRESSURE_MIN_BAR||rear>legacy::PRESSURE_MAX_BAR)throw ArgumentError("tyre pressure must lie in [0.8,3.0] bar");
    options->tyre=options->tyre.with_pressures(front,rear);
    if(args.has("gearing")) {
        auto gear=args.text("gearing");std::ranges::replace(gear,'X','x');const auto x=gear.find('x');
        if(x==std::string::npos)throw ArgumentError("--gearing must be CHAINRINGxCOG");
        numbers.set("front",gear.substr(0,x));numbers.set("rear",gear.substr(x+1));
        const auto f=numbers.integer("front"),r=numbers.integer("rear");
        if(f<1||f>1000||r<1||r>1000)throw ArgumentError("gearing teeth out of range");
        options->drivetrain.chainring_teeth=static_cast<int>(f);options->drivetrain.cog_teeth=static_cast<int>(r);options->drivetrain.auto_shift=false;
    }
    options->drivetrain.ripple_depth=args.real("ripple_depth",.85);options->drivetrain.crank_phase_deg=args.real("crank_phase",0.);options->drivetrain.validate();
    extras=Value{Object{{"rider_mass_kg",real(options->setup.model.rider.value().total_rider_mass())}}}; // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
    if(seed)set(extras,"seed",real(io::as_double(Value{io::Number{*seed}},"seed")));
    if(args.has("sag")) {
        if(!options->setup.model.rider.value().present())throw ArgumentError("--sag requires a rider"); // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
        if(!args.flag("headless"))std::cerr<<"[bike-ride] --sag applies to headless runs; the viewer uses the shipped tune.\n";
        else {
            const double target=args.real("sag");
            if(target<=0.||target>=100.)throw ArgumentError("sag percentage must lie strictly between 0 and 100");
            auto& bike=options->setup.model.bike;
            const auto tuning=model::compute_suspension_tuning_for_sag(bike,options->setup.model.masses,&options->setup.model.rider.value(),{},target,target,{}); // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
            bike.fork_initial_psi=tuning.fork_calibrated_psi;bike.fork_air_tokens=tuning.fork_air_tokens;bike.shock_stiffness=tuning.shock_stiffness_n_m;
            set(extras,"sag_target_pct",real(target));set(extras,"fork_psi",real(bike.fork_initial_psi));set(extras,"coil_rate_n_m",real(bike.shock_stiffness));
            std::cout<<"[bike-ride] --sag "<<geometry::detail::py_g(target)<<"%: fork "<<geometry::detail::py_fixed(bike.fork_initial_psi,1)<<" psi ("<<bike.fork_air_tokens<<" tokens), coil "<<geometry::detail::py_fixed(bike.shock_stiffness,0)<<" N/m\n";
        }
    }
    return options;
}
[[gnu::noinline]] int legacy_headless(const std::unique_ptr<legacy::RideSimulation>& runtime,
    const legacy::RideOptions& options, const Arguments& args, io::Value& extras,
    const std::optional<std::string>& seed, platform::InterruptShield const& interrupt) {
    const auto initial=runtime->telemetry();
    const auto lane=[&](std::string_view name){const auto found=std::ranges::find(legacy::kLegacyChannels,name);if(found==legacy::kLegacyChannels.end())throw std::logic_error("missing legacy telemetry lane");return initial[static_cast<std::size_t>(found-legacy::kLegacyChannels.begin())];};
    set(extras,"start_fork_travel_mm",real(lane("fork_travel_mm")));set(extras,"start_shock_stroke_mm",real(lane("shock_stroke_mm")));
    for(const auto& [key,value]:Object{{"drive_mode",text(options.drive_mode)},{"assist",text(options.drive_mode=="pedelec"?options.assist_mode:"off")},
        {"legs",text(options.legs.value())},{"visual_pedalling",Value{options.visual_pedalling}},{"gear_ratio",real(options.drivetrain.gear_ratio())}, // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
        {"ripple_depth",real(options.drivetrain.ripple_depth)},{"tyre_model",text(options.tyre.model)},{"tyre_tier",text(options.tyre.tier)},
        {"tyre_pressure_front_bar",real(options.tyre.front.pressure_bar)},{"tyre_pressure_rear_bar",real(options.tyre.rear.pressure_bar)},
        {"surface",text(options.tyre.surface.value_or(options.setup.track.surface))}})set(extras,key,value);
    const auto decimate=args.integer("decimate",1);if(decimate<1)throw ArgumentError("--decimate must be positive");
    legacy::RideRecorder recorder(runtime->timestep_s(),static_cast<std::size_t>(decimate));recorder.record(*runtime);
    const auto limits=runtime->default_limits();const double started=platform::monotonic_seconds();std::string reason;
    for(;;){
        if(interrupt.deferred()){reason="operator_stop";break;}
        if(const auto ended=legacy::termination_reason(limits,runtime->position_m(),runtime->steps(),platform::monotonic_seconds()-started,runtime->crash_event())){reason=*ended;break;}
        runtime->step();recorder.record(*runtime);
    }
    const legacy::RunOutcome outcome{.reason=reason,.steps=runtime->steps(),.sim_time_s=runtime->time_s(),.wall_clock_s=platform::monotonic_seconds()-started,.position_m=runtime->position_m(),.crash=runtime->crash_event()};
    legacy::SummaryOptions summary_options;summary_options.target_speed_kmh=options.setup.runtime.speed_kph;summary_options.start_x_m=runtime->start_x_m();
    summary_options.fork_travel_mm=options.setup.model.bike.fork_travel;summary_options.shock_stroke_mm=options.setup.model.bike.shock_stroke;
    summary_options.shock_bumper_engage_mm=runtime->shock_bumper_engage_mm();summary_options.extras=io::as_object(extras,"legacy extras");summary_options.tyre_history=recorder.tyre_history();
    summary_options.rider_variant=options.setup.model.rider.value().variant;summary_options.rider_mass_kg=options.setup.model.rider.value().mass_kg; // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
    auto recorded=recorder.columns();
    const auto summary=legacy::summarize_ride(recorded,recorder.sample_interval_s(),options.setup.track,outcome,summary_options);
    const auto root=std::filesystem::path(args.text("out","output/ride"))/legacy_directory_name(options,seed);
    std::filesystem::create_directories(root);recorder.write_csv(root/"telemetry.csv");summary.write_json(root/"summary.json");
    if(!args.flag("no_plots")) {
        viz::Channels columns;for(auto& [name,values]:recorded)columns.emplace_back(name,std::move(values));
        std::vector<unsigned char> window;for(const double x:*viz::numeric(columns,"x_m"))window.push_back(x>=runtime->start_x_m()+legacy::kRampExclusionM?1:0);
        (void)viz::plot_ride(columns,recorder.sample_interval_s(),options.setup.track,options.setup.model.bike.fork_travel,options.setup.model.bike.shock_stroke,root,window);
        viz::write_artifact(viz::track_profile(options.setup.track),root/"profile.png");
        if(options.tyre.pneumatic())viz::write_artifact(viz::tyre_plot(columns,options.setup.track),root/"tyres.png");
    }
    std::cout<<summary.format_table()<<"\n[bike-ride] wrote "<<root<<'\n';return interrupt.deferred()?130:0;
}
int legacy_run(std::unique_ptr<ResolvedRide> resolved) {
    const auto& args=resolved->arguments;
    const auto seed=ride_track_seed(args);
    io::Value extras;
    auto const options=legacy_options(*resolved,seed,extras);
    platform::InterruptShield interrupt;
    // make_ride consumes an owned copy; the caller's options stay readable below.
    auto runtime=legacy::make_ride(std::make_unique<legacy::RideOptions>(*options));
    if(!args.flag("headless")){
        viewer::ViewerOptions view;view.mode=viewer::Mode::legacy;view.timestep_s=runtime->timestep_s();view.time_scale=static_cast<unsigned>(args.integer("time_scale",1));
        viewer::LifecycleHooks lifecycle;lifecycle.close_requested=[&]{return interrupt.deferred()!=0;};
        (void)viewer::run_legacy(*runtime,view,lifecycle);return interrupt.deferred()?130:0;
    }
    return legacy_headless(runtime,*options,args,extras,seed,interrupt);
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
        if (request->setup.model.physics->spatial) {
            request->sensors_v2.sample_period_s = input.real("sensor_period",.005);
            request->sensors_v2.latency_s = input.real("sensor_delay",.01);
            request->sensors_v2.maximum_read_period_s = request->experiment.control_period_s;
            request->sensors_v2.validate();
        }
        request->road_resolution_m=input.real("road_resolution",.005);
        request->programs=research::load_programs(input.has("rider_program")?std::optional<std::filesystem::path>(input.text("rider_program")):std::nullopt,{});
        if(input.has("demand"))request->programs.demand_frames=research::constant_demand(input.real("demand")).demand_frames;
        request->capture_mode=input.flag("record")?runtime::ResearchCaptureMode::full:runtime::ResearchCaptureMode::off;
        auto args=resolved->arguments;args.set("assist",input.has("demand")?"false":"true");
        return run_research(std::move(request),args,!input.flag("headless"));
    }
    return resolved->setup.model.physics.value().physics_mode=="physical"?physical_run(std::move(resolved)):legacy_run(std::move(resolved)); // NOLINT(bugprone-unchecked-optional-access) request/parse schema guarantees engagement; .value() is the loud contract
}
} // namespace bike::cli
