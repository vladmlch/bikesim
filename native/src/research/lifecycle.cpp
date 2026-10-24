#include "lifecycle.hpp"
#include "evidence.hpp"
#include "../platform/provenance.hpp"
#include "../platform/signals.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace bike::research {
OperatorInterrupted::~OperatorInterrupted() = default;
ResearchPublication::~ResearchPublication() = default;
namespace {
using namespace runtime::setupvalue;
std::string exception_text(const std::exception_ptr& failure) {
    try { if (failure) std::rethrow_exception(failure); }
    catch (const std::exception& error) { return error.what(); }
    catch (...) { return "non-standard exception"; }
    return {};
}
void clock_input(double now, std::int64_t step) {
    if (!std::isfinite(now) || step < 0) throw std::invalid_argument("clock requires finite time and nonnegative completed step");
}
bool is_interrupt(const std::exception_ptr& error) {
    try { if (error) std::rethrow_exception(error); }
    catch (const OperatorInterrupted&) { return true; }
    catch (...) { return false; }
    return false;
}
} // namespace

void FailureReport::add(std::string_view operation, std::exception_ptr failure) noexcept {
    if (!primary) primary = std::move(failure);
    // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
    else { try { secondary.push_back(std::string(operation) + ": " + exception_text(failure)); } catch (...) {} }
}
void FailureReport::note(std::string_view message) noexcept {
    // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
    try { secondary.emplace_back(message); } catch (...) {}
}
void FailureReport::rethrow_if_failed() const { if (primary) std::rethrow_exception(primary); }
void drain_active(PolicySession& session) {
    session.request_stop();
    while (session.pending()) (void)session.advance_pending(.008);
}
void drain_active(runtime::NativeResearchRuntime& environment) {
    environment.stop();
    for (;;) {
        const auto status = environment.status();
        const bool active = runtime::sample_wire::boolean(runtime::sample_wire::required(status, "active"));
        const bool pending = runtime::sample_wire::boolean(runtime::sample_wire::required(status, "delivery_pending"));
        if (!active && !pending) return;
        if (environment.advance_control(.008)) environment.acknowledge_control();
    }
}
void close_environment(PolicySession& session, FailureReport& failure) {
    try { session.close(); }
    catch (...) { failure.add("research close failed", std::current_exception()); }
    try { failure.secondary.insert(failure.secondary.end(), session.cleanup_notes().begin(), session.cleanup_notes().end()); }
    catch (...) { failure.add("copying cleanup diagnostics failed", std::current_exception()); }
}

ResearchRun::ResearchRun(PolicySession& session, std::shared_ptr<ResearchPublication> publication)
    : session_(&session), environment_(&session.environment()), publication_(std::move(publication)) {
    if (publication_ && session.environment().configuration().capture_mode != runtime::ResearchCaptureMode::full)
        throw std::invalid_argument("persistent research recording requires full capture before setup");
    event("episode_started");
    session_->set_reset_hooks([this] { before_reset(); }, [this] { new_episode(); });
}
ResearchRun::ResearchRun(runtime::NativeResearchRuntime& environment, std::shared_ptr<ResearchPublication> publication)
    : environment_(&environment), publication_(std::move(publication)) {
    if (publication_ && environment.configuration().capture_mode != runtime::ResearchCaptureMode::full)
        throw std::invalid_argument("persistent research recording requires full capture before setup");
    event("episode_started");
}
ResearchRun::~ResearchRun() { if (session_) session_->set_reset_hooks({}, {}); }
void ResearchRun::event(std::string kind, io::Value values) {
    if (!publication_) return;
    const auto status = environment_->status();
    events_.push_back(OperatorEvent{.event_id=static_cast<std::int64_t>(events_.size()),
        .processed_step=runtime::sample_wire::integer(runtime::sample_wire::required(status, "step")),
        .time_s=runtime::sample_wire::number(runtime::sample_wire::required(status, "time_s")),
        .received_wall_s=platform::monotonic_seconds(), .kind=std::move(kind), .queued=false, .terminal=false, .applied=false, .values=std::move(values), .episode_id=episode_id_});
}
void ResearchRun::before_reset() {
    if (finalization_attempted_ && !draining_) throw std::runtime_error("cannot reset a finalized research launch");
    const auto status = environment_->status();
    const auto& reason = runtime::sample_wire::required(status, "reason");
    (void)seal(runtime::sample_wire::is_none(reason) ? "reset" : runtime::sample_wire::string(reason));
}
void ResearchRun::new_episode() {
    ++episode_id_; prepared_.reset(); prepared_events_.reset();
    event("episode_started", Value{Object{{"reset", Value{true}}}});
}
void ResearchRun::annotate_failure() {
    if (!failures_.primary || is_interrupt(failures_.primary)) return;
    const auto annotate = [&](EpisodeReceipt& receipt) {
        auto episode = receipt.episode;
        set(episode.run_metadata, "launch_failure", Value{Object{{"error", text(exception_text(failures_.primary))}}});
        auto summary = summarize(episode);
        set(summary, "seal_reason", text(receipt.seal_reason));
        receipt.episode = std::move(episode);
        receipt.summary = std::move(summary);
    };
    if (prepared_) annotate(*prepared_);
    for (auto& receipt : episodes_)
        if (receipt.episode_id == episode_id_) annotate(receipt);
}
void ResearchRun::reset(std::optional<std::string> seed) {
    if (session_) {
        if (session_->pending()) throw std::runtime_error("drain the accepted control window before resetting a recorded episode");
        session_->reset(std::move(seed));
        return;
    }
    auto config = environment_->configuration();
    const auto chosen = seed ? validate_seed(*seed) : config.seed_decimal;
    const auto status = environment_->status();
    if (runtime::sample_wire::boolean(runtime::sample_wire::required(status, "active")) ||
        runtime::sample_wire::boolean(runtime::sample_wire::required(status, "delivery_pending")))
        throw std::runtime_error("drain the accepted control window before resetting a recorded episode");
    std::optional<runtime::ResearchNoiseTape> tape;
    std::optional<runtime::ResearchSensorState> state;
    auto metadata = reseed_metadata(config.metadata, chosen);
    auto assets = reseed_assets(config.assets, metadata);
    if(config.raw_initial_v2&&config.sensors_v2) {
        auto generated=make_sensor_tape_v2(*config.sensors_v2,chosen,1+config.max_steps/config.sensor_steps);
        runtime::spatial::SensorPipelineV2 pipeline(*config.sensors_v2,generated);
        pipeline.reset(*config.raw_initial_v2);
        (void)pipeline.read(0.);
        state=pipeline.state();tape=std::move(generated);
    } else if (config.raw_initial) {
        SensorConfig sensors;
        sensors.delivery = config.sensors;
        sensors.sample_period_s = static_cast<double>(config.sensor_steps) * config.timestep_s;
        sensors.noise = {.acceleration_std_mps2=config.noise_std[0], .gyro_std_rad_s=config.noise_std[1], .encoder_std_rad_s=config.noise_std[2], .torque_std_nm=config.noise_std[3]};
        tape = make_sensor_tape(sensors, chosen, 1 + config.max_steps / config.sensor_steps);
        state = startup_sensor_state(sensors, std::get<runtime::NoiseTape>(*tape), *config.raw_initial);
    } else if (chosen != config.seed_decimal) {
        throw std::invalid_argument("a captured bootstrap without raw startup sensors cannot be reseeded");
    }
    before_reset();
    if (tape) environment_->reset(std::move(*tape), std::move(*state), chosen, std::move(metadata), std::move(assets)); // NOLINT(bugprone-unchecked-optional-access) a captured state accompanies every tape
    else environment_->reset();
    new_episode();
}
const EpisodeReceipt* ResearchRun::seal(std::string reason) {
    if (!publication_) return nullptr;
    if (reason.empty()) throw std::invalid_argument("episode seal reason must be nonempty");
    for (const auto& receipt : episodes_)
        if (receipt.episode_id == episode_id_) return &receipt;
    if (!prepared_) {
        runtime::OwnedResearchEpisode episode;
        const auto capture = [&] {
            if (session_) return session_->seal_episode();
            auto owned = environment_->seal_episode();
            set(owned.run_metadata, "identity_checks", identity_receipt(owned.configuration.metadata));
            return owned;
        };
        try { episode = capture(); }
        catch (const runtime::InvalidReferenceRun&) {
            // A strict accounting flush commits its failure prefix first.
            // Only the ownership step is retried: never integrate a suffix.
            episode = capture();
        }
        if (failures_.primary && !is_interrupt(failures_.primary))
            set(episode.run_metadata, "launch_failure", Value{Object{{"error", text(exception_text(failures_.primary))}}});
        set(episode.run_metadata, "seal_reason", text(reason));
        auto summary = summarize(episode);
        set(summary, "seal_reason", text(reason));
        prepared_ = EpisodeReceipt{.episode_id=episode_id_, .seal_reason=std::move(reason), .episode=std::move(episode), .summary=std::move(summary)};
    }
    if (!prepared_events_) {
        std::vector<OperatorEvent> staged;
        for (const auto& event : events_)
            if (event.episode_id == episode_id_) staged.push_back(event);
        if (session_) {
            const auto& operator_events = session_->operator_events();
            staged.insert(staged.end(), operator_events.begin(), operator_events.end());
        }
        const auto& final = prepared_->episode.final_snapshot;
        if (final.first_failure)
            staged.push_back(OperatorEvent{.event_id=0, .processed_step=final.step, .time_s=final.time_s, .received_wall_s=platform::monotonic_seconds(),
                .kind="model_first_failure", .queued=false, .terminal=false, .applied=false,
                .values=Value{Object{{"evidence", wire_value(final.first_failure->as_wire())}, {"classification", text("measured")}}}, .episode_id=episode_id_});
        staged.push_back(OperatorEvent{.event_id=0, .processed_step=final.step, .time_s=final.time_s, .received_wall_s=platform::monotonic_seconds(),
            .kind="episode_ended", .queued=false, .terminal=true, .applied=false,
            .values=Value{Object{{"reason", text(prepared_->seal_reason)}, {"episode_id", integer(episode_id_)}}}, .episode_id=episode_id_});
        std::ranges::stable_sort(staged, [](const auto& first, const auto& second) {
            return first.received_wall_s < second.received_wall_s;
        });
        prepared_events_ = std::move(staged);
    }
    // Reserve both owners before the receipt and its terminating event are
    // committed, so an allocation retry cannot invent a duplicate episode.
    episodes_.reserve(episodes_.size() + 1);
    events_.reserve(events_.size() + prepared_events_->size());
    episodes_.push_back(std::move(*prepared_)); prepared_.reset();
    std::erase_if(events_, [this](const auto& event) { return event.episode_id == episode_id_; });
    for (auto& event : *prepared_events_) {
        event.event_id = static_cast<std::int64_t>(events_.size());
        event.episode_id = episode_id_;
        events_.push_back(std::move(event));
    }
    prepared_events_.reset();
    return &episodes_.back();
}
std::optional<std::filesystem::path> ResearchRun::finish(std::exception_ptr primary, bool stop) {
    if (path_) return path_;
    if (finalizing_) throw std::runtime_error("research finalization is already in progress");
    const bool first = !finalization_attempted_;
    finalization_attempted_ = true;
    finalizing_ = true;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members) aggregate mirrors the Python reference structure
    struct ScopeExit { bool& active; explicit ScopeExit(bool& flag) : active(flag) {} ~ScopeExit() { active = false; }
        ScopeExit(const ScopeExit&) = delete;
        ScopeExit& operator=(const ScopeExit&) = delete;
        ScopeExit(ScopeExit&&) = delete;
        ScopeExit& operator=(ScopeExit&&) = delete;
    };
    const ScopeExit finalizing{finalizing_};
    if (first) failures_.primary = std::move(primary);
    if (first && stop) {
        draining_ = true;
        try { if (session_) drain_active(*session_); else drain_active(*environment_); }
        catch (...) { failures_.add("active control drain failed", std::current_exception()); }
        draining_ = false;
    }
    platform::InterruptShield const shield;
    if (!final_reason_) {
        const auto status = environment_->status();
        const auto& reason = runtime::sample_wire::required(status, "reason");
        final_reason_ = runtime::sample_wire::is_none(reason)
            ? is_interrupt(failures_.primary) ? "operator_stop"
              : failures_.primary ? "simulation_error" : stop ? "operator_stop" : "viewer_closed"
            : runtime::sample_wire::string(reason);
    }
    try {
        annotate_failure();
        if (failures_.primary && !failure_event_recorded_) {
            event(is_interrupt(failures_.primary) ? "launch_interrupted" : "launch_failed",
                Value{Object{{"error", text(exception_text(failures_.primary))}}});
            failure_event_recorded_ = true;
        }
        (void)seal(*final_reason_); // NOLINT(bugprone-unchecked-optional-access) final_reason_ is assigned before publication begins
        if (shield.deferred()) {
            try { event("operator_stop_deferred", Value{Object{{"deferred_during", text("publication")}}}); }
            // NOLINTNEXTLINE(bugprone-empty-catch) deliberate: the result is advisory and the failure path is handled separately
            catch (...) { /* Observability cannot break the shielded publish. */ }
        }
        if (publication_) path_ = publication_->publish(episodes_, events_);
    } catch (...) {
        const auto failure = std::current_exception();
        if (shield.deferred()) failures_.note("operator interrupt deferred during research publication");
        if (failures_.primary) {
            failures_.add("recording finalization failed", failure);
            failures_.rethrow_if_failed();
        }
        std::rethrow_exception(failure);
    }
    if (shield.deferred()) {
        if (!failures_.primary) throw OperatorInterrupted{};
        failures_.note("operator interrupt deferred during research publication");
    }
    failures_.rethrow_if_failed();
    return path_;
}

PlaybackClock::PlaybackClock(double timestep) : timestep_s_(timestep) {
    if (!std::isfinite(timestep) || timestep <= 0.) throw std::invalid_argument("timestep_s must be positive and finite");
}
void PlaybackClock::rebase(double now, std::int64_t step) {
    clock_input(now, step); wall_s_ = now; step_ = step; debt_s_ = 0.; initialized_ = true;
}
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
void PlaybackClock::set_scale(int scale, double now, std::int64_t step) {
    if (scale != 1 && scale != 2 && scale != 4 && scale != 8) throw std::invalid_argument("scale must be one of 1, 2, 4, 8");
    clock_input(now, step); scale_ = scale; rebase(now, step);
}
void PlaybackClock::set_paused(bool paused, double now, std::int64_t step) {
    clock_input(now, step);
    if (paused_ != paused) { paused_ = paused; rebase(now, step); }
}
std::int64_t PlaybackClock::target(double now, std::int64_t step) {
    clock_input(now, step);
    if (!initialized_) rebase(now, step);
    if (step < step_) throw std::invalid_argument("completed step moved backwards; rebase after reset");
    const auto completed = step - step_;
    double debt = std::max(0., debt_s_ - static_cast<double>(completed) * timestep_s_);
    const double elapsed = std::max(0., now - wall_s_);
    if (paused_) debt = 0.;
    else debt = std::min(debt + elapsed * static_cast<double>(scale_), std::max(.05 * static_cast<double>(scale_), timestep_s_));
    const double more = std::floor(debt / timestep_s_);
    if (!std::isfinite(more) || more >= static_cast<double>(std::numeric_limits<std::int64_t>::max() - step))
        throw std::overflow_error("pacing target exceeds step range");
    debt_s_ = debt; step_ = step; wall_s_ = std::max(wall_s_, now);
    return step + static_cast<std::int64_t>(more);
}
} // namespace bike::research
