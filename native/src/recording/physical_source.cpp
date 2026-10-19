#include "contracts.hpp"
#include "../runtime/encode.hpp"
#include "value_util.hpp"
#include "../research/evidence.hpp"
#include <algorithm>
#include <limits>

namespace bike::recording {
namespace {
using io::Value;
using Object = Value::Object;
bool optimized(const FieldSpec& f) {
    if (f.dtype == "map<bigint>") return false;
    return std::ranges::all_of(f.children, optimized);
}
class NativePhysicalSource final : public PhysicalSource {
public:
    ~NativePhysicalSource() override;
    NativePhysicalSource(const NativePhysicalSource&) = delete;
    NativePhysicalSource& operator=(const NativePhysicalSource&) = delete;
    NativePhysicalSource(NativePhysicalSource&&) = delete;
    NativePhysicalSource& operator=(NativePhysicalSource&&) = delete;
    NativePhysicalSource(std::shared_ptr<const runtime::OwnedNativeEpisode> episode, Catalog catalog, bool ordinary)
        : episode_(std::move(episode)), catalog_(std::move(catalog)), ordinary_(ordinary) {
        if (!episode_) throw std::invalid_argument("missing owned physical episode");
        if (episode_->generation < 0 || episode_->samples.size() > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max())) throw std::invalid_argument("invalid owned physical extent");
        for (const auto& sample : episode_->samples) if (!sample) throw std::invalid_argument("null retained physical sample");
    }
    std::int64_t row_count() const override { return static_cast<std::int64_t>(episode_->samples.size()); }
    const Catalog* catalog() const noexcept override { return &catalog_; }
    bool ordinary_commands() const noexcept override { return ordinary_; }
    void tables(std::size_t maximum, const BatchSink& sink) const override {
        if (!maximum || maximum > static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max())) throw std::invalid_argument("batch_rows must be positive int64");
        const auto& specs = catalog_.table_order();
        if (episode_->samples.empty()) {
            for (const auto& spec : specs) sink(spec.name, encode_rows(spec, {}, episode_->generation));
            return;
        }
        std::vector<std::optional<runtime::EncodePlan>> plans;
        for (const auto& spec : specs) {
            if (std::ranges::all_of(spec.fields, optimized)) plans.emplace_back(compile_plan(catalog_, spec.name));
            else plans.emplace_back();
        }
        for (std::size_t first = 0; first < episode_->samples.size();) {
            const auto count = std::min(maximum, episode_->samples.size()-first);
            if (ordinary_) {
                std::vector<Object> rows;
                for (const auto& command : runtime::applied_command_rows_native(*episode_, static_cast<std::int64_t>(first), static_cast<std::int64_t>(count))) {
                    Object row{{"application_id", value::integer(command.application_id)}, {"time_s", value::real(command.time_s)}, {"step", value::integer(command.step)}, {"control", research::wire_value(command.control)}};
                    if (command.front_brake_demand) row.emplace_back("front_brake_demand", research::wire_value(*command.front_brake_demand));
                    if (command.rear_brake_demand) row.emplace_back("rear_brake_demand", research::wire_value(*command.rear_brake_demand));
                    if (command.rollback_brake_demand) row.emplace_back("rollback_brake_demand", research::wire_value(*command.rollback_brake_demand));
                    rows.push_back(std::move(row));
                }
                if (!rows.empty()) sink("commands_applied", encode_rows(table_spec("commands_applied"), rows, episode_->generation));
            }
            for (std::size_t i = 0; i < specs.size(); ++i) {
                const auto& spec = specs[i];
                if (auto& plan = plans[i]) {
                    auto batch = runtime::encode_table_native(*episode_, plan.value(), static_cast<std::int64_t>(first), static_cast<std::int64_t>(count));
                    validate_pieces(batch, spec); piece_fragments(spec.name, std::move(batch), maximum, sink); continue;
                }
                // Reviewed slow source adapter for a future catalog dtype the
                // optimized projector does not implement (currently map<bigint>).
                // Unknown roots still reject before a projection can omit them.
                std::vector<Object> rows;
                for (std::size_t n = first; n < first+count; ++n) {
                    const auto& sample = *episode_->samples[n];
                    for (const auto& [root, v] : sample.channels) { (void)v; (void)catalog_.channel_table(root); }
                    Object row{{"interval_id", value::integer(sample.interval_id)}};
                    for (const auto& root : spec.claims) if (const auto* v = runtime::sample_wire::find(sample.channels, root)) row.emplace_back(root, research::wire_value(*v));
                    rows.push_back(std::move(row));
                }
                sink(spec.name, encode_rows(spec, rows, episode_->generation));
            }
            first += count;
        }
    }
private:
    std::shared_ptr<const runtime::OwnedNativeEpisode> episode_;
    Catalog catalog_;
    bool ordinary_;
};
NativePhysicalSource::~NativePhysicalSource() = default;
} // namespace
std::shared_ptr<const PhysicalSource> make_physical_source(runtime::OwnedNativeEpisode episode, Catalog catalog, bool ordinary) {
    return std::make_shared<const NativePhysicalSource>(std::make_shared<const runtime::OwnedNativeEpisode>(std::move(episode)), std::move(catalog), ordinary);
}
std::shared_ptr<const PhysicalSource> make_physical_source(std::shared_ptr<const runtime::OwnedNativeEpisode> episode, Catalog catalog, bool ordinary) {
    return std::make_shared<const NativePhysicalSource>(std::move(episode), std::move(catalog), ordinary);
}
} // namespace bike::recording
