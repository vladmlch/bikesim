#pragma once
#include "catalog.hpp"
#include "encoding.hpp"
#include "../runtime/research.hpp"
#include <filesystem>
#include <memory>
#include <optional>
#include <span>

namespace bike::recording {
struct RunMetadata {
    std::string run_uuid, started_utc;
    std::string backend = "native", mode = "ride";
    io::Value values{io::Value::Object{}};
    double wall_origin_s = 0.;
    void validate() const;
};
[[nodiscard]] RunMetadata make_run_metadata(std::string backend, std::string mode, io::Value values);
struct InputEvent {
    std::int64_t episode_id = 0, event_id = 0;
    std::string kind;
    double received_wall_s = 0.;
    std::int64_t processed_step = 0;
    std::optional<std::int64_t> applied_step;
    io::Value values{io::Value::Object{}};
    void validate() const;
};
struct Asset {
    std::string name, media_type, sha256;
    std::vector<std::byte> content;
    void validate() const;
};
[[nodiscard]] Asset asset(std::string name, std::string media_type, std::vector<std::byte> content);
[[nodiscard]] Asset json_asset(std::string name, const io::Value& value);
[[nodiscard]] std::vector<Asset> replace_recipe(std::vector<Asset>, const io::Value& recipe);
// End-of-launch diagnostics only. Re-read declared input paths for their
// current identities; these bytes never replace the captured Asset owners.
[[nodiscard]] io::Value input_identity_at_finalization(const io::Value& metadata);

class PhysicalSource {
public:
    virtual ~PhysicalSource();
    PhysicalSource() = default;
    PhysicalSource(const PhysicalSource&) = delete;
    PhysicalSource& operator=(const PhysicalSource&) = delete;
    PhysicalSource(PhysicalSource&&) = delete;
    PhysicalSource& operator=(PhysicalSource&&) = delete;
    [[nodiscard]] virtual std::int64_t row_count() const = 0;
    [[nodiscard]] virtual const Catalog* catalog() const noexcept = 0;
    [[nodiscard]] virtual bool ordinary_commands() const noexcept { return false; }
    virtual void tables(std::size_t maximum_rows, const BatchSink&) const = 0;
};
class ResearchSource {
public:
    virtual ~ResearchSource();
    ResearchSource() = default;
    ResearchSource(const ResearchSource&) = delete;
    ResearchSource& operator=(const ResearchSource&) = delete;
    ResearchSource(ResearchSource&&) = delete;
    ResearchSource& operator=(ResearchSource&&) = delete;
    [[nodiscard]] virtual std::int64_t row_count(std::string_view stream) const = 0;
    virtual void tables(std::size_t maximum_rows, const BatchSink&) const = 0;
};
struct ResearchEvidence {
    std::shared_ptr<const ResearchSource> history;
    std::vector<double> initial_integration_state, final_integration_state;
    int replay_contract_version = recording::replay_contract_version;
    std::int64_t complete_control_steps = 0;
    bool replay_eligible = false;
    void validate() const;
};
struct EpisodeEvidence {
    std::int64_t episode_id = 0, generation = 0;
    std::shared_ptr<const PhysicalSource> physical;
    io::Value summary{io::Value::Object{}};
    std::vector<Asset> assets;
    std::optional<ResearchEvidence> research;
    void validate() const;
};
[[nodiscard]] std::shared_ptr<const PhysicalSource> make_physical_source(
    runtime::OwnedNativeEpisode episode, Catalog catalog, bool ordinary_commands = false);
[[nodiscard]] std::shared_ptr<const PhysicalSource> make_physical_source(
    std::shared_ptr<const runtime::OwnedNativeEpisode> episode, Catalog catalog, bool ordinary_commands = false);
[[nodiscard]] std::shared_ptr<const ResearchSource> make_research_source(
    std::shared_ptr<const runtime::OwnedResearchEpisode> episode);
[[nodiscard]] std::shared_ptr<const EpisodeEvidence> make_research_evidence(
    std::int64_t episode_id, runtime::OwnedResearchEpisode episode, Catalog catalog, io::Value summary);
// Source-owned ForceSample value: construction validates then snapshots all
// vectors. No mutable aliases into a live engine or caller survive.
class ForceSample {
public:
    ForceSample(double time_s, std::vector<double> qpos, std::vector<double> qvel,
                runtime::ForceComponents components);
    [[nodiscard]] double time_s() const noexcept { return time_s_; }
    [[nodiscard]] const std::vector<double>& qpos() const noexcept { return qpos_; }
    [[nodiscard]] const std::vector<double>& qvel() const noexcept { return qvel_; }
    [[nodiscard]] const runtime::ForceComponents& components() const noexcept { return components_; }
    [[nodiscard]] std::vector<std::pair<std::string,double>> component_powers() const;
private:
    double time_s_;
    std::vector<double> qpos_, qvel_;
    runtime::ForceComponents components_;
};
} // namespace bike::recording
