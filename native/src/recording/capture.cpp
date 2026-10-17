#include "capture.hpp"
#include "value_util.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <limits>
#include <random>
#include <set>

namespace bike::recording {
namespace {
void text_required(std::string_view value, std::string_view label) {
    if (value.empty()) throw std::invalid_argument("recording: " + std::string(label) + " must be nonempty");
}
void object_required(const io::Value& v, std::string_view label) {
    (void)io::as_object(v, label);
    (void)io::recording_json(v); // rejects duplicate keys/malformed tags before ownership changes
}
bool lower_hex(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); }
void nonnegative(std::int64_t n, std::string_view label) { if (n < 0) throw std::invalid_argument("recording: negative " + std::string(label)); }
} // namespace
PhysicalSource::~PhysicalSource() = default;
ResearchSource::~ResearchSource() = default;
void RunMetadata::validate() const {
    if (run_uuid.size() != 36) throw std::invalid_argument("run_uuid must be a UUID");
    for (std::size_t i = 0; i < run_uuid.size(); ++i) {
        const bool dash = i == 8 || i == 13 || i == 18 || i == 23;
        if (dash ? run_uuid[i] != '-' : !lower_hex(run_uuid[i])) throw std::invalid_argument("run_uuid must be canonical lowercase UUID");
    }
    if (started_utc.size() < 20 || started_utc[4] != '-' || started_utc[7] != '-' ||
        (started_utc[10] != 'T' && started_utc[10] != ' ') || started_utc[13] != ':' || started_utc[16] != ':' ||
        !(started_utc.ends_with('Z') || started_utc.ends_with("+00:00"))) throw std::invalid_argument("started_utc must be an ISO UTC datetime");
    // The shared tagged codec checks calendar/range validity without consulting
    // a timezone database or normalizing an invalid user timestamp silently.
    (void)io::recording_json(io::Value{io::Tagged{"datetime", {{"value", started_utc.ends_with('Z') ? started_utc.substr(0, started_utc.size()-1) + "+00:00" : started_utc}}}});
    text_required(backend, "backend"); text_required(mode, "mode"); object_required(values, "run metadata");
    if (!std::isfinite(wall_origin_s)) throw std::invalid_argument("wall_origin_s must be finite");
}
RunMetadata make_run_metadata(std::string backend, std::string mode, io::Value values) {
    std::random_device entropy;
    std::array<unsigned char,16> bytes{};
    for (auto& b : bytes) b = static_cast<unsigned char>(entropy() & 255U);
    bytes[6] = static_cast<unsigned char>((bytes[6] & 15U) | 64U); bytes[8] = static_cast<unsigned char>((bytes[8] & 63U) | 128U);
    constexpr std::string_view digits = "0123456789abcdef";
    std::string uuid;
    for (std::size_t i = 0; i < bytes.size(); ++i) { if (i == 4 || i == 6 || i == 8 || i == 10) uuid += '-'; uuid += digits[bytes[i] >> 4U]; uuid += digits[bytes[i] & 15U]; }
    const auto now = std::chrono::system_clock::now();
    const auto whole = std::chrono::floor<std::chrono::seconds>(now);
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now - whole).count();
    const auto time = std::chrono::system_clock::to_time_t(whole);
    std::tm utc{};
    if (!gmtime_r(&time, &utc)) throw std::runtime_error("recording UTC time unavailable");
    char stamp[32]{};
    if (std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &utc) == 0) throw std::runtime_error("recording UTC format failed");
    auto sub = std::to_string(micros); sub.insert(0, 6 - sub.size(), '0');
    RunMetadata out{std::move(uuid), std::string(stamp) + "." + sub + "+00:00", std::move(backend), std::move(mode), std::move(values),
        std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()};
    out.validate(); return out;
}
void InputEvent::validate() const {
    nonnegative(episode_id, "episode_id"); nonnegative(event_id, "event_id"); nonnegative(processed_step, "processed_step");
    text_required(kind, "kind");
    if (!std::isfinite(received_wall_s)) throw std::invalid_argument("received_wall_s must be finite");
    if (applied_step && *applied_step < processed_step) throw std::invalid_argument("applied_step precedes processed_step");
    object_required(values, "event.values");
}
void Asset::validate() const {
    text_required(name, "asset name"); text_required(media_type, "asset media_type");
    if (sha256.size() != 64 || !std::ranges::all_of(sha256, lower_hex)) throw std::invalid_argument("asset sha256 must be lowercase hex");
}
void ResearchEvidence::validate() const {
    if (!history || replay_contract_version != recording::replay_contract_version || complete_control_steps < 0 || initial_integration_state.size() != final_integration_state.size())
        throw std::invalid_argument("invalid research evidence ownership/state/version");
}
void EpisodeEvidence::validate() const {
    nonnegative(episode_id, "episode_id"); nonnegative(generation, "generation");
    if (!physical || physical->row_count() < 0) throw std::invalid_argument("physical evidence owner/extent invalid");
    object_required(summary, "episode summary");
    std::set<std::string> names;
    for (const auto& a : assets) { a.validate(); if (!names.insert(a.name).second) throw std::invalid_argument("duplicate episode asset " + a.name); }
    if (research) research->validate();
}
RunCapture::RunCapture(RunMetadata metadata, bool enabled) : metadata_(std::move(metadata)), enabled_(enabled) { metadata_.validate(); }
std::string_view RunCapture::state_name() const noexcept {
    switch (state_) { case State::collecting: return "collecting"; case State::publishing: return "publishing"; case State::published: return "published"; case State::publication_failed: return "publication_failed"; }
    return "unknown";
}
void RunCapture::require_collecting(std::string_view what) const {
    if (state_ != State::collecting) throw std::logic_error("cannot record " + std::string(what) + ": run capture is " + std::string(state_name()));
}
void RunCapture::event(InputEvent event) {
    event.validate(); require_collecting("event");
    const auto key = std::pair{event.episode_id, event.event_id};
    if (last_event_ && key <= *last_event_) throw std::invalid_argument("event identity does not follow previous event");
    if (enabled_) events_.push_back(std::move(event));
    last_event_ = key;
}
void RunCapture::add_episode(std::shared_ptr<const EpisodeEvidence> evidence) {
    if (!evidence) throw std::invalid_argument("null episode evidence");
    evidence->validate(); require_collecting("episode");
    if (!enabled_) return;
    for (std::size_t i = 0; i < episodes_.size(); ++i) if (episodes_[i]->episode_id == evidence->episode_id) {
        if (episode_origins_[i].lock() == evidence) return;
        throw std::invalid_argument("conflicting evidence for episode " + std::to_string(evidence->episode_id));
    }
    if (!episodes_.empty() && (episodes_.back()->episode_id == std::numeric_limits<std::int64_t>::max() || evidence->episode_id != episodes_.back()->episode_id + 1))
        throw std::invalid_argument("episode identifier breaks prefix continuity");
    // Freeze metadata/assets at the capture boundary even when the caller kept
    // a mutable shared_ptr alias. Source histories are immutable source owners,
    // and retain shared const SamplePtrs instead of copying the numeric history.
    auto snapshot = std::make_shared<const EpisodeEvidence>(*evidence);
    episode_origins_.push_back(evidence);
    try { episodes_.push_back(std::move(snapshot)); } catch (...) { episode_origins_.pop_back(); throw; }
}
EpisodeJournal::EpisodeJournal(RunCapture& capture, std::int64_t episode_id) : capture_(&capture), episode_id_(episode_id) { nonnegative(episode_id, "journal episode_id"); }
InputEvent EpisodeJournal::emit(std::string kind, double received, std::int64_t processed, std::optional<std::int64_t> applied, io::Value values) {
    if (next_id_ == std::numeric_limits<std::int64_t>::max()) throw std::overflow_error("recording journal event identity exhausted");
    InputEvent event{episode_id_, next_id_, std::move(kind), received, processed, applied, std::move(values)};
    capture_->event(event); ++next_id_; return event;
}
void EpisodeJournal::emit_all(std::span<const InputEvent> events) {
    for (const auto& event : events) {
        if (event.episode_id != episode_id_ || event.event_id != next_id_ || next_id_ == std::numeric_limits<std::int64_t>::max()) throw std::invalid_argument("event does not continue journal");
        capture_->event(event); ++next_id_;
    }
}
ForceSample::ForceSample(double time, std::vector<double> qpos, std::vector<double> qvel, runtime::ForceComponents components)
    : time_s_(time), qpos_(std::move(qpos)), qvel_(std::move(qvel)), components_(std::move(components)) {
    if (!std::isfinite(time_s_) || !std::ranges::all_of(qpos_, [](double v) { return std::isfinite(v); }) || !std::ranges::all_of(qvel_, [](double v) { return std::isfinite(v); })) throw std::invalid_argument("force sample state shape or value mismatch");
    std::set<std::string> names;
    for (const auto& [name, forces] : components_) if (forces.size() != qvel_.size() || !std::ranges::all_of(forces, [](double v) { return std::isfinite(v); }) || !names.insert(name).second) throw std::invalid_argument("force sample shape or value mismatch");
}
std::vector<std::pair<std::string,double>> ForceSample::component_powers() const {
    std::vector<std::pair<std::string,double>> out;
    for (const auto& [name, forces] : components_) out.emplace_back(name, runtime::sample_wire::dot(forces, qvel_));
    return out;
}
} // namespace bike::recording
