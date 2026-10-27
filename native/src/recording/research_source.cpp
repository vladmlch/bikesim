#include "contracts.hpp"
#include "versioned.hpp"
#include "value_util.hpp"
#include "../research/evidence.hpp"
#include "../runtime/spatial_checkpoint.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace bike::recording {
namespace {
class NativeResearchSource final : public ResearchSource {
public:
    ~NativeResearchSource() override;
    NativeResearchSource(const NativeResearchSource&) = delete;
    NativeResearchSource& operator=(const NativeResearchSource&) = delete;
    NativeResearchSource(NativeResearchSource&&) = delete;
    NativeResearchSource& operator=(NativeResearchSource&&) = delete;
    explicit NativeResearchSource(std::shared_ptr<const runtime::OwnedResearchEpisode> episode) : episode_(std::move(episode)) { if (!episode_) throw std::invalid_argument("missing research source owner"); }
    std::int64_t row_count(std::string_view name) const override {
        const auto count = episode_->row_count(name);
        if (count > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max())) throw std::overflow_error("research row count exceeds int64");
        return static_cast<std::int64_t>(count);
    }
    void tables(std::size_t maximum, const BatchSink& sink) const override {
        if (!maximum || maximum > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max())) throw std::invalid_argument("batch_rows must be positive int64");
        const auto version = !episode_->observations.empty() && std::holds_alternative<runtime::spatial::SensorObservationV2>(episode_->observations.front())
            ? CatalogVersion::spatial2 : CatalogVersion::planar1;
        for (const auto& spec : all_tables(version)) {
            if (spec.name != "commands_requested" && spec.name != "commands_applied" && spec.name != "observations" && spec.name != "transitions") continue;
            const auto count = episode_->row_count(spec.name);
            for (std::size_t first = 0; first < count;) {
                const auto size = std::min(maximum, count-first);
                std::vector<io::Value::Object> rows;
                const auto batch = episode_->batch(spec.name, static_cast<std::int64_t>(first), static_cast<std::int64_t>(size));
                for (std::size_t i = 0; i < batch.size(); ++i) {
                    auto staged = research::wire_value(batch[i]); auto row = std::move(std::get<io::Value::Object>(staged.data));
                    const auto identity = static_cast<std::int64_t>(first) + static_cast<std::int64_t>(i);
                    if (spec.name == "transitions") {
                        std::erase_if(row, [](const auto& p) { return p.first == "observation"; });
                        value::set(row, "request_id", value::integer(identity));
                        value::set(row, "previous_observation_id", value::integer(identity));
                        value::set(row, "observation_id", value::integer(identity+1));
                    }
                    value::set(row, spec.keys.at(1).key, value::integer(identity)); rows.push_back(std::move(row));
                }
                sink(spec.name, encode_rows(spec, rows, 0)); first += size;
            }
        }
    }
private:
    std::shared_ptr<const runtime::OwnedResearchEpisode> episode_;
};
NativeResearchSource::~NativeResearchSource() = default;
} // namespace
std::shared_ptr<const ResearchSource> make_research_source(std::shared_ptr<const runtime::OwnedResearchEpisode> episode) {
    return std::make_shared<const NativeResearchSource>(std::move(episode));
}
std::shared_ptr<const EpisodeEvidence> make_research_evidence(std::int64_t id, runtime::OwnedResearchEpisode episode, Catalog catalog, io::Value summary) {
    if (episode.configuration.capture_mode != runtime::ResearchCaptureMode::full) throw std::invalid_argument("persistent research recording requires full capture");
    auto const owner = std::make_shared<const runtime::OwnedResearchEpisode>(std::move(episode));
    auto physical_owner = std::shared_ptr<const runtime::OwnedNativeEpisode>(owner, &owner->physical);
    const auto version = catalog_version(catalog);
    auto physical = make_physical_source(std::move(physical_owner), std::move(catalog), false);
    bool eligible = false;
    if (const auto* extents = io::find_member(io::as_object(summary, "research summary"), "recording_extents"))
        if (const auto* flag = io::find_member(io::as_object(*extents, "recording_extents"), "replay_eligible")) eligible = io::as_bool(*flag, "replay_eligible");
    eligible = eligible && owner->requested.size() == owner->transitions.size() && owner->observations.size() == owner->transitions.size()+1 &&
        std::cmp_equal(owner->physical.samples.size(), owner->final_snapshot.step) &&
        std::ranges::all_of(owner->initial_integration_state, [](double n) { return std::isfinite(n); }) &&
        std::ranges::all_of(owner->final_snapshot.integration_state, [](double n) { return std::isfinite(n); });
    std::vector<Asset> assets;
    if (owner->configuration.assets) for (const auto& a : *owner->configuration.assets) assets.push_back(Asset{.name=a.name, .media_type=a.media_type, .sha256=a.sha256, .content=a.content});
    if(version==CatalogVersion::spatial2) {
        const auto initial=runtime::spatial::decode_checkpoint(owner->checkpoint_initial);
        assets.push_back(json_asset("spatial/checkpoint_initial.json",owner->checkpoint_initial));
        if(owner->checkpoint_unavailable_reason) {
            if(!io::is_null(owner->checkpoint_final))throw std::invalid_argument("unavailable checkpoint cannot also advertise complete state");
            eligible=false;
            assets.push_back(json_asset("spatial/checkpoint_final.json",io::Value{io::Value::Object{
                {"spatial_checkpoint_unavailable",value::integer(1)},{"replayable",io::Value{false}},
                {"reason",value::text(*owner->checkpoint_unavailable_reason)}}}));
        } else {
        (void)runtime::spatial::decode_checkpoint(owner->checkpoint_final);
        assets.push_back(json_asset("spatial/checkpoint_final.json",owner->checkpoint_final));
        const auto source=std::ranges::find_if(assets,[](const auto& a){return a.name=="spatial/terrain_source.json";});
        if(source==assets.end())throw std::invalid_argument("spatial research recording lacks terrain source");
        std::string terrain_source;terrain_source.reserve(source->content.size());
        for(const std::byte byte:source->content)terrain_source.push_back(static_cast<char>(std::to_integer<unsigned char>(byte)));
        const auto document=io::parse_recording_json(terrain_source);
        const auto kind=io::as_string(value::require(document,"kind"),"terrain kind");
        const io::Value recipe{io::Value::Object{{"replay_contract_version",value::integer(4)},
            {"terrain_kind",value::text(kind)},{"terrain_sha256",value::require(initial.identity,"terrain_sha256")},
            {"config_spatial",value::require(summary,"resolved_config")},{"checkpoint_initial",owner->checkpoint_initial},
            {"checkpoint_final",owner->checkpoint_final},{"execution_identity_sha256",value::require(initial.identity,"solver")}}};
        assets=replace_recipe(std::move(assets),recipe);
        }
    }
    auto out = std::make_shared<const EpisodeEvidence>(EpisodeEvidence{.episode_id=id, .generation=owner->physical.generation, .physical=std::move(physical), .summary=std::move(summary), .assets=std::move(assets),
        .research=ResearchEvidence{.history=make_research_source(owner), .initial_integration_state=owner->initial_integration_state, .final_integration_state=owner->final_snapshot.integration_state,
            .replay_contract_version=version == CatalogVersion::spatial2 ? 4 : replay_contract_version, .complete_control_steps=static_cast<std::int64_t>(owner->transitions.size()), .replay_eligible=eligible}});
    out->validate(); return out;
}
} // namespace bike::recording
