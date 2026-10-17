#include "application.hpp"
#include "physical_capture.hpp"
#include "../io/json.hpp"
#include "../platform/provenance.hpp"
#include "../platform/signals.hpp"
#include "../recording/research_publication.hpp"
#include "../research/evidence.hpp"
#include "../research/lifecycle.hpp"
#include "../rider/state_codec.hpp"
#include "../runtime/setup_values.hpp"
#include "../viewer/session.hpp"
#include <algorithm>
#include <iostream>
#include <memory>

namespace bike::cli {
namespace {
using namespace runtime::setupvalue;
bool status_flag(const runtime::NativeResearchRuntime& environment, std::string_view name) {
    const auto status = environment.status();
    return runtime::sample_wire::boolean(runtime::sample_wire::required(status, name));
}
bool terminal(const runtime::NativeResearchRuntime& environment) {
    const auto status = environment.status();
    return runtime::sample_wire::boolean(runtime::sample_wire::required(status, "terminated")) ||
           runtime::sample_wire::boolean(runtime::sample_wire::required(status, "truncated"));
}
bool pending(const runtime::NativeResearchRuntime& environment) {
    return status_flag(environment, "active") || status_flag(environment, "delivery_pending");
}
runtime::RideControl fixed_command(const runtime::NativeResearchRuntime& environment, const Arguments& args) {
    // The CLI's fixed command owns rider inputs. Motor policies retain the
    // narrower PolicySession contract; this path does not pretend to be one.
    runtime::RideControl control;
    const auto status = environment.status();
    const auto& demand = runtime::sample_wire::required(status, "demand_nm");
    if (!args.flag("assist")) control.motor_torque_nm = runtime::sample_wire::is_none(demand)
        ? args.real("motor_torque", 80.) : runtime::sample_wire::number(demand);
    control.motor_limit_nm = args.optional_real("motor_limit");
    const bool owned = !environment.configuration().programs.rider_frames.empty();
    if (!owned) {
        if (args.present("human_torque") || !args.has("physics_config"))
            control.human_torque_nm = args.real("human_torque", 0.);
        const auto name = args.text("posture", "neutral");
        if (name != "neutral") {
            const double phase = std::clamp((environment.snapshot().time_s - .5) / .5, 0., 1.);
            const double blend = phase * phase * (3. - 2. * phase);
            ::rider::RiderPosture posture;
            if (name == "forward") { posture.torso_lean_rad = .20 * blend; posture.pelvis_pitch_rad = .05 * blend; }
            else if (name == "crouched") { posture.torso_lean_rad = .10 * blend; posture.pelvis_offset_m = spindle::Vec2{.02 * blend, -.06 * blend}; }
            else { posture.torso_lean_rad = .08 * blend; posture.pelvis_offset_m = spindle::Vec2{0., .16 * blend}; posture.use_saddle = blend < .1; }
            control.posture = posture;
        }
    }
    // Reuse the public wire validator for scalar and posture domains.
    return ::rider::decode_intent_control(::rider::encode_intent_control(control), "CLI command");
}
std::string error_text(const std::exception_ptr& failure) {
    try { if (failure) std::rethrow_exception(failure); }
    catch (const std::exception& error) { return error.what(); }
    catch (...) { return "non-standard exception"; }
    return {};
}
} // namespace

int run_research(research::ResearchRequest request, const Arguments& args, bool interactive) {
    platform::InterruptShield interrupt;
    std::unique_ptr<runtime::NativeResearchRuntime> owned_environment;
    std::unique_ptr<research::PolicySession> session;
    std::unique_ptr<research::ResearchRun> launch;
    runtime::NativeResearchRuntime* environment = nullptr;
    std::optional<std::filesystem::path> output;
    std::exception_ptr failure;
    bool interrupted = false;
    int result = 2;
    try {
        auto prepared = research::prepare_environment(std::move(request));
        std::shared_ptr<research::ResearchPublication> publication;
        if (args.flag("record")) {
            auto metadata = prepared.metadata;
            set(metadata, "command", text(args.command));
            publication = recording::make_research_publication(
                recording::make_run_metadata("native", interactive ? "research_viewer" : "research_headless", metadata),
                model_catalog(prepared.bootstrap.ride.model), args.text("out", "output/research"));
        }
        owned_environment = runtime::make_research(std::move(prepared.bootstrap));
        environment = owned_environment.get();
        launch = std::make_unique<research::ResearchRun>(*environment, publication);
        auto policy_name = args.optional_text("policy");
        if (!policy_name && args.command == "bike-ride")
            policy_name = "bike_sim.sim.research.policies:passthrough_factory";
        if (policy_name) {
            try {
                auto policy = research::load_policy(*policy_name);
                session = std::make_unique<research::PolicySession>(std::move(owned_environment), std::move(policy));
            } catch (...) {
                try { environment->fail_policy(error_text(std::current_exception())); } catch (...) {}
                throw;
            }
            launch = std::make_unique<research::ResearchRun>(*session, publication);
        }
        const auto warn = [&] {
            if (const auto warning = environment->take_pending_warning()) std::cerr << "WARNING: " << *warning << '\n';
        };
        const auto stop = [&] {
            if (session) research::drain_active(*session); else research::drain_active(*environment);
        };
        if (interactive) {
            viewer::ViewerOptions options;
            options.mode = viewer::Mode::research; options.title = "bike-ride research";
            options.timestep_s = environment->configuration().timestep_s;
            options.time_scale = static_cast<unsigned>(args.integer("time_scale", 1));
            if (session) {
                viewer::LifecycleHooks lifecycle;
                lifecycle.close_requested = [&] { if (interrupt.deferred()) interrupted = true; return interrupted; };
                lifecycle.after_advance = [&] { warn(); if (interrupt.deferred()) { interrupted = true; stop(); } };
                (void)viewer::run_research(*session, options, lifecycle);
            } else {
                double front = 0., rear = 0.;
                bool paused = false;
                viewer::SessionHooks hooks;
                hooks.close_requested = [&] { if (interrupt.deferred()) interrupted = true; return interrupted; };
                hooks.frame = [&] { return environment->render_frame(); };
                hooks.terminal = [&] { return terminal(*environment); };
                hooks.advance = [&](std::int64_t target, double budget) {
                    if (interrupt.deferred()) { interrupted = true; stop(); return; }
                    if (terminal(*environment) || paused) return;
                    if (!pending(*environment)) environment->begin_control(fixed_command(*environment, args), front, rear);
                    if (environment->advance_control(budget, target)) environment->acknowledge_control();
                    warn();
                };
                hooks.stop = stop;
                hooks.pause = [&](bool value) {
                    while (pending(*environment)) if (environment->advance_control(.008)) environment->acknowledge_control();
                    paused = value; launch->event(value ? "pause" : "resume");
                };
                hooks.brakes = [&](double f, double r) {
                    front = f; rear = r;
                    launch->event("brakes", Value{Object{{"front", real(f)}, {"rear", real(r)}}});
                };
                hooks.reset = [&] {
                    while (pending(*environment)) if (environment->advance_control(.008)) environment->acknowledge_control();
                    launch->reset(); front = rear = 0.; paused = false;
                };
                (void)viewer::run(std::move(hooks), environment->model_image(), options);
            }
        } else {
            while (!terminal(*environment)) {
                if (interrupt.deferred()) { interrupted = true; break; }
                if (session) {
                    if (!session->pending()) session->begin_advance();
                    (void)session->advance_pending(.008);
                } else {
                    if (!pending(*environment)) environment->begin_control(fixed_command(*environment, args), 0., 0.);
                    if (environment->advance_control(.008)) environment->acknowledge_control();
                }
                warn();
            }
        }
        if (interrupted) throw research::OperatorInterrupted{};
        output = launch->finish({}, true);
        const auto status = environment->status();
        result = research::episode_exit_code(status);
        std::cout << io::recording_json(Value{Object{
            {"output", output ? text(output->string()) : Value{}},
            {"recording", text(output ? "published" : "off")},
            {"outcome", research::wire_value(runtime::sample_wire::required(status, "reason"))},
            {"metrics", research::wire_value(runtime::sample_wire::required(status, "metrics"))}}}) << '\n';
    } catch (...) {
        failure = std::current_exception();
        try { std::rethrow_exception(failure); } catch (const research::OperatorInterrupted&) { interrupted = true; } catch (...) {}
        if (launch && !launch->finalization_attempted()) {
            try { output = launch->finish(failure, true); }
            catch (...) { /* The launch retains primary and secondary diagnostics. */ }
        }
        std::cerr << "ERROR: " << error_text(failure) << '\n';
        if (launch) for (const auto& note : launch->failures().secondary) std::cerr << "  " << note << '\n';
        result = interrupted || interrupt.deferred() ? 130 : environment ? 3 : 2;
    }
    research::FailureReport cleanup;
    cleanup.primary = failure;
    if (session) research::close_environment(*session, cleanup);
    else if (environment) {
        try { environment->close(); } catch (...) { cleanup.add("research close failed", std::current_exception()); }
    }
    for (const auto& note : cleanup.secondary) std::cerr << "  " << note << '\n';
    if (!failure && cleanup.primary) { std::cerr << "ERROR: " << error_text(cleanup.primary) << '\n'; result = 3; }
    return interrupt.deferred() ? 130 : result;
}

int research_command(const Arguments& args) {
    try { return run_research(resolve_research(args), args, false); }
    catch (const std::exception& error) { std::cerr << "ERROR: " << error.what() << '\n'; return 2; }
}
} // namespace bike::cli
