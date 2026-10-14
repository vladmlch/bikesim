#include "samples.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "../cblas_abi.hpp"

namespace runtime {
namespace sample_wire {

const Wire *find(const WireObject &value, std::string_view key) {
    for (const auto &[name, item] : value)
        if (name == key) return &item;
    return nullptr;
}

const Wire &required(const WireObject &value, std::string_view key) {
    if (const Wire *item = find(value, key)) return *item;
    throw std::invalid_argument("accounting: missing field '" +
                                std::string(key) + "'");
}

const WireObject &object(const Wire &value) {
    if (const auto *result = std::get_if<WireObject>(&value.value)) return *result;
    throw std::invalid_argument("accounting: expected an object");
}

const WireArray &array(const Wire &value) {
    if (const auto *result = std::get_if<WireArray>(&value.value)) return *result;
    throw std::invalid_argument("accounting: expected an array");
}

double number(const Wire &value) {
    double result = 0.;
    if (const auto *real = std::get_if<double>(&value.value)) result = *real;
    else if (const auto *whole = std::get_if<std::int64_t>(&value.value))
        result = static_cast<double>(*whole);
    else throw std::invalid_argument("accounting: expected a finite number");
    if (!std::isfinite(result))
        throw std::invalid_argument("accounting: expected a finite number");
    return result;
}

std::int64_t integer(const Wire &value) {
    if (const auto *result = std::get_if<std::int64_t>(&value.value)) return *result;
    throw std::invalid_argument("accounting: expected an integer");
}

bool boolean(const Wire &value) {
    if (const auto *result = std::get_if<bool>(&value.value)) return *result;
    throw std::invalid_argument("accounting: expected a boolean");
}

const std::string &string(const Wire &value) {
    if (const auto *result = std::get_if<std::string>(&value.value)) return *result;
    throw std::invalid_argument("accounting: expected a string");
}

bool is_none(const Wire &value) noexcept {
    return std::holds_alternative<std::monostate>(value.value);
}

Wire strings(std::span<const std::string> values) {
    WireArray result;
    result.reserve(values.size());
    for (const auto &value : values) result.emplace_back(value);
    return Wire{std::move(result)};
}

void set(WireObject &object, std::string key, Wire value) {
    for (auto &[name, slot] : object)
        if (name == key) {
            slot = std::move(value);
            return;
        }
    object.emplace_back(std::move(key), std::move(value));
}

void merge(WireObject &destination, const WireObject &source) {
    for (const auto &[key, value] : source) set(destination, key, value);
}

// NOLINTNEXTLINE(cppcoreguidelines-rvalue-reference-param-not-moved) moves each element, not the container
void merge(WireObject &destination, WireObject &&source) {
    for (auto &[key, value] : source)
        set(destination, std::move(key), std::move(value));
}

void exact(const WireObject &value, std::span<const std::string_view> keys,
           std::string_view path) {
    if (value.size() != keys.size())
        throw std::invalid_argument(std::string(path) + ": wrong field set");
    for (const auto key : keys)
        if (find(value, key) == nullptr)
            throw std::invalid_argument(std::string(path) + "." +
                                        std::string(key) + ": missing field");
}

double dot(std::span<const double> a, std::span<const double> b) {
    if (a.size() != b.size() || a.size() > static_cast<std::size_t>(
                                                   std::numeric_limits<int>::max()))
        throw std::invalid_argument("accounting: incompatible force/velocity widths");
    if (a.empty()) return 0.;
    const double result = blas::ddot(static_cast<int>(a.size()), a.data(), 1,
                                     b.data(), 1);
    if (!std::isfinite(result))
        throw std::invalid_argument("accounting: nonfinite interval power");
    return result;
}

// Contiguous float64 reduction: eight lanes, scalar tail, pairwise recursion.
// NOLINTNEXTLINE(misc-no-recursion) bounded logarithmic reduction depth
double numpy_sum(std::span<const double> values) {
    const std::size_t n = values.size();
    if (n < 8) {
        double result = -0.;
        for (const double value : values) result += value;
        return result;
    }
    if (n <= 128) {
        std::array<double, 8> lanes{};
        std::ranges::copy(values.first(8), lanes.begin());
        std::size_t i = 8;
        for (; i + 8 <= n; i += 8)
            for (std::size_t j = 0; j < 8; ++j) lanes[j] += values[i + j];
        double result = ((lanes[0] + lanes[1]) + (lanes[2] + lanes[3])) +
                        ((lanes[4] + lanes[5]) + (lanes[6] + lanes[7]));
        for (; i < n; ++i) result += values[i];
        return result;
    }
    std::size_t half = n / 2;
    half -= half % 8;
    return numpy_sum(values.first(half)) + numpy_sum(values.subspan(half));
}

} // namespace sample_wire
namespace {
using namespace sample_wire;

void finite(double value, std::string_view path, bool nonnegative = false) {
    if (!std::isfinite(value) || (nonnegative && value < 0.))
        throw std::invalid_argument(std::string(path) + ": invalid finite value");
}

// Index suffixes go through a stack buffer — no std::to_string temporary.
void append_index(std::string &out, std::size_t index) {
    std::array<char, 20> digits{};
    char *const end = std::to_chars(digits.data(), digits.end(), index).ptr;
    out.append(digits.data(),
               static_cast<std::size_t>(std::distance(digits.data(), end)));
}

// `prefix` is the caller's scratch: children extend it in place and restore it
// on the way out, so a leaf costs one string copy instead of a fresh nested
// temporary per level.
// NOLINTNEXTLINE(misc-no-recursion) recursion follows an owned, finite Wire tree
void flatten(const Wire &value, std::string &prefix,
             FlatColumns &destination) {
    if (const auto *fields = std::get_if<WireObject>(&value.value)) {
        for (const auto &[key, item] : *fields) {
            const std::size_t base = prefix.size();
            if (!prefix.empty()) prefix += '.';
            prefix += key;
            flatten(item, prefix, destination);
            prefix.resize(base);
        }
    } else if (const auto *items = std::get_if<WireArray>(&value.value)) {
        for (std::size_t i = 0; i < items->size(); ++i) {
            const std::size_t base = prefix.size();
            prefix += '.';
            append_index(prefix, i);
            flatten((*items)[i], prefix, destination);
            prefix.resize(base);
        }
    } else if (const auto *real = std::get_if<double>(&value.value)) {
        destination.emplace_back(prefix, *real);
    } else if (const auto *whole = std::get_if<std::int64_t>(&value.value)) {
        destination.emplace_back(prefix, static_cast<double>(*whole));
    } else if (const auto *flag = std::get_if<bool>(&value.value)) {
        destination.emplace_back(prefix, *flag ? 1. : 0.);
    }
}

// std::map::emplace keeps the first inserted value. The stable sort holds
// traversal order inside each equal-name run, so keeping a run's first
// entry reproduces the flatten-into-map dedup exactly.
void keep_first(FlatColumns &row) {
    const auto by_name = [](const auto &a, const auto &b) {
        return a.first < b.first;
    };
    std::ranges::stable_sort(row, by_name);
    const auto tail = std::ranges::unique(
        row, [](const auto &a, const auto &b) {
            return a.first == b.first;
        });
    row.erase(tail.begin(), row.end());
}

// Sequential map::operator[] writes overwrite. The stable sort holds
// statement order inside each equal-name run, so a run's last entry wins.
void keep_last(FlatColumns &row) {
    const auto by_name = [](const auto &a, const auto &b) {
        return a.first < b.first;
    };
    std::ranges::stable_sort(row, by_name);
    std::size_t kept = 0;
    for (std::size_t i = 0; i < row.size();) {
        std::size_t next = i + 1;
        while (next < row.size() && row[next].first == row[i].first) ++next;
        // Move only across positions: in-place runs must not self-move-assign.
        if (kept != next - 1) row[kept] = std::move(row[next - 1]);
        ++kept;
        i = next;
    }
    row.resize(kept);
}

constexpr std::array<std::string_view, 2> kSides{"front", "rear"};
constexpr std::array<std::string_view, 3> kThresholdNames{"0.0", "1.0", "5.0"};
constexpr std::array<double, 3> kThresholds{0., 1., 5.};

} // namespace

WireObject PhysicalSampleData::as_wire() const {
    WireObject result{{"schema_version", Wire(2)},
                      {"interval_id", Wire(interval_id)},
                      {"time_s", Wire(time_s)},
                      {"interval_end_s", Wire(end_time_s)},
                      {"dt_s", Wire(dt_s())},
                      {"qpos", wire_array(qpos)},
                      {"qvel", wire_array(qvel)},
                      {"powers_w", Wire(powers_w)}};
    sample_wire::merge(result, channels);
    return result;
}

FlatColumns
PhysicalSampleData::flat_columns(const ColumnLayout &layout) const {
    // flatten(as_wire()) without owning the tree: the eight fixed fields
    // keep their positions, and a same-named channels entry supplies the
    // flattened value for that slot (merge writes the last matching source
    // value into the kept slot). The remaining channels fields follow in
    // first-seen order, again holding each key's last value.
    constexpr std::array<std::string_view, 8> fields{
        "schema_version", "interval_id", "time_s", "interval_end_s",
        "dt_s", "qpos", "qvel", "powers_w"};
    std::array<const Wire *, fields.size()> overrides{};
    std::vector<const WireObject::value_type *> extras;
    extras.reserve(channels.size());
    for (const auto &field : channels) {
        std::size_t slot = fields.size();
        for (std::size_t i = 0; i < fields.size(); ++i)
            if (field.first == fields[i]) {
                slot = i;
                break;
            }
        if (slot != fields.size()) {
            overrides[slot] = &field.second;
            continue;
        }
        const auto same = std::ranges::find_if(extras, [&](const auto *entry) {
            return entry->first == field.first;
        });
        if (same == extras.end()) extras.push_back(&field);
        else *same = &field;
    }
    FlatColumns row;
    row.reserve(qpos.size() + qvel.size() + powers_w.size() +
                channels.size() + 16);
    std::string scratch;
    const auto fixed = [&](std::size_t slot, const char *name) {
        if (const Wire *value = overrides[slot]) {
            scratch.assign(name);
            flatten(*value, scratch, row);
            return true;
        }
        return false;
    };
    if (!fixed(0, "schema_version")) row.emplace_back("schema_version", 2.);
    if (!fixed(1, "interval_id"))
        row.emplace_back("interval_id", static_cast<double>(interval_id));
    if (!fixed(2, "time_s")) row.emplace_back("time_s", time_s);
    if (!fixed(3, "interval_end_s"))
        row.emplace_back("interval_end_s", end_time_s);
    if (!fixed(4, "dt_s")) row.emplace_back("dt_s", dt_s());
    if (!fixed(5, "qpos"))
        for (std::size_t i = 0; i < qpos.size(); ++i) {
            scratch.assign("qpos.");
            append_index(scratch, i);
            row.emplace_back(scratch, qpos[i]);
        }
    if (!fixed(6, "qvel"))
        for (std::size_t i = 0; i < qvel.size(); ++i) {
            scratch.assign("qvel.");
            append_index(scratch, i);
            row.emplace_back(scratch, qvel[i]);
        }
    if (!fixed(7, "powers_w"))
        for (const auto &[name, power] : powers_w) {
            scratch.assign("powers_w.");
            scratch += name;
            flatten(power, scratch, row);
        }
    for (const auto *field : extras) {
        scratch.assign(field->first);
        flatten(field->second, scratch, row);
    }
    keep_first(row);
    // The explicit alias writes below land exactly where the old code's
    // map::operator[] assignments did: every listed name overwrites or
    // inserts, statement order deciding repeats.
    FlatColumns assigned;
    assigned.reserve(qpos.size() + qvel.size() + 2 * powers_w.size() + 16);
    assigned.emplace_back("x_m", qpos.at(layout.root_x_qpos));
    assigned.emplace_back("speed_mps", qvel.at(layout.root_x_dof));
    assigned.emplace_back("pitch_rad", qpos.at(layout.root_pitch_qpos));
    assigned.emplace_back("interval_start_s", time_s);
    assigned.emplace_back("interval_dt_s", dt_s());
    for (std::size_t i = 0; i < qpos.size(); ++i) {
        scratch.assign("prestep_qpos_");
        append_index(scratch, i);
        assigned.emplace_back(scratch, qpos[i]);
    }
    for (std::size_t i = 0; i < qvel.size(); ++i) {
        scratch.assign("prestep_qvel_");
        append_index(scratch, i);
        assigned.emplace_back(scratch, qvel[i]);
    }
    for (const auto &[name, power] : powers_w) {
        scratch.assign("power_");
        scratch += name;
        scratch += "_w";
        assigned.emplace_back(scratch, number(power));
        scratch.assign(name);
        scratch += "_power_w";
        assigned.emplace_back(scratch, number(power));
    }
    if (const auto *work = find(channels, "component_work_j"))
        for (const auto &[name, value] : object(*work)) {
            scratch.assign(name);
            scratch += "_work_j";
            assigned.emplace_back(scratch, number(value));
        }
    const auto &mass = object(required(channels, "mass"));
    assigned.emplace_back("compiled_mass_kg",
                          number(required(mass, "mass_kg")));
    const auto &com = array(required(mass, "com_m"));
    constexpr std::array<std::string_view, 3> axes{"x", "y", "z"};
    for (std::size_t i = 0; i < axes.size(); ++i) {
        scratch.assign("compiled_com_");
        scratch += axes[i];
        scratch += "_m";
        assigned.emplace_back(scratch, number(com.at(i)));
    }
    // Decimated rows carry the reduced {mass_kg, com_m} mass channel —
    // absent leaves surface as NaN in the merged column, exactly like
    // PhysicalRecorder's flat.get(key, nan) (physical_recorder.py:133).
    for (const auto name : {"kinetic_energy_j", "gravitational_energy_j"})
        if (const Wire *value = find(mass, name))
            assigned.emplace_back(name, number(*value));
    const auto &energy = object(required(channels, "energy"));
    assigned.emplace_back("mechanical_energy_j",
                          number(required(energy, "mechanical_energy_j")));
    assigned.emplace_back("energy_residual_j",
                          number(required(energy, "residual_j")));
    keep_last(assigned);
    // Overlay the sorted rows: on a shared name the explicit write wins,
    // exactly like operator[] on the flattened map.
    FlatColumns result;
    result.reserve(row.size() + assigned.size());
    auto leaf = row.begin();
    auto extra = assigned.begin();
    while (leaf != row.end() && extra != assigned.end()) {
        if (extra->first < leaf->first) {
            result.push_back(std::move(*extra));
            ++extra;
        } else if (leaf->first < extra->first) {
            result.push_back(std::move(*leaf));
            ++leaf;
        } else {
            result.push_back(std::move(*extra));
            ++extra;
            ++leaf;
        }
    }
    for (; leaf != row.end(); ++leaf) result.push_back(std::move(*leaf));
    for (; extra != assigned.end(); ++extra) result.push_back(std::move(*extra));
    return result;
}

SampleColumns make_columns(std::span<const SamplePtr> samples,
                           const ColumnLayout &layout) {
    SampleColumns result;
    const double missing = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto flat = samples[i]->flat_columns(layout);
        // The flat row and the map are both name-sorted: one forward walk
        // replaces a per-leaf tree lookup, and a fresh column inserts just
        // before the walk position. An empty column still marks the insert.
        auto slot = result.begin();
        for (const auto &[key, value] : flat) {
            while (slot != result.end() && slot->first < key) ++slot;
            if (slot == result.end() || slot->first != key)
                slot = result.try_emplace(slot, key);
            if (slot->second.empty())
                slot->second.assign(samples.size(), missing);
            slot->second[i] = value;
        }
    }
    return result;
}

OwnedNativeEpisode OwnedNativeEpisode::seal(
    int generation, std::span<const SamplePtr> retained,
    const ColumnLayout &columns) {
    return {.generation = generation,
            .samples = std::vector<SamplePtr>(retained.begin(),
                                              retained.end()),
            .columns = columns};
}

RuntimeSampleBatch OwnedNativeEpisode::sample_batch(std::int64_t start,
                                             std::int64_t limit) const {
    const auto size = static_cast<std::int64_t>(samples.size());
    if (start < 0 || start > size)
        throw std::invalid_argument("episode batch start is outside the retained extent");
    if (limit < 1)
        throw std::invalid_argument("episode batch limit must be positive");
    const auto last = start + std::min(limit, size - start);
    std::vector<SamplePtr> slice(samples.begin() + start,
                                 samples.begin() + last);
    return {.generation = generation, .samples = std::move(slice), .columns = {}};
}

RuntimeSampleBatch OwnedNativeEpisode::batch(std::int64_t start, std::int64_t limit) const {
    auto result = sample_batch(start, limit);
    result.columns = make_columns(result.samples, columns);
    return result;
}

void WorkHistory::add(const PhysicalSampleData &sample) {
    if (last_id == std::numeric_limits<std::int64_t>::max())
        throw std::overflow_error("physical history interval ID overflow");
    if (sample.interval_id < 0 || (last_id.has_value() &&
        (sample.interval_id != *last_id + 1 || !last_end.has_value() ||
         std::abs(sample.time_s - *last_end) > 1e-10)))
        throw std::invalid_argument("missing, overlapping or repeated force interval");
    finite(sample.time_s, "sample.time_s", true);
    finite(sample.end_time_s, "sample.end_time_s", true);
    if (sample.dt_s() <= 0.)
        throw std::invalid_argument("force interval must have positive duration");
    // Validate both wheels before changing any accumulator.
    const auto &tires = object(required(sample.channels, "tires"));
    std::array<double, 2> loads{};
    for (std::size_t side = 0; side < kSides.size(); ++side) {
        const auto &tire = object(required(tires, kSides[side]));
        for (const auto &patch_value : array(required(tire, "patches"))) {
            const auto &patch = object(patch_value);
            const double normal = number(required(patch, "normal_load_n"));
            finite(normal, "raw normal load", true);
            const auto &source = string(required(patch, "source_geom"));
            if (source != "terrain" && source != "catch_plane")
                throw std::invalid_argument("unrecognized physical contact source");
            if (source == "terrain") loads[side] += normal;
        }
        finite(loads[side], "total working-road load", true);
    }
    WireObject updated = work_j;
    for (const auto &[name, power] : sample.powers_w) {
        const double increment = number(power) * sample.dt_s();
        finite(increment, "interval work");
        const Wire *previous = find(updated, name);
        const double value = (previous == nullptr ? 0. : number(*previous)) + increment;
        finite(value, "accumulated work");
        set(updated, name, Wire(value));
    }
    const double duration = duration_s + sample.dt_s();
    finite(duration, "accumulated duration", true);
    auto airtime = airtime_s;
    for (std::size_t side = 0; side < kSides.size(); ++side)
        for (std::size_t threshold = 0; threshold < kThresholds.size(); ++threshold)
            if (loads[side] <= kThresholds[threshold]) {
                airtime[side][threshold] += sample.dt_s();
                finite(airtime[side][threshold], "accumulated airtime", true);
            }
    work_j = std::move(updated);
    airtime_s = airtime;
    duration_s = duration;
    last_id = sample.interval_id;
    last_end = sample.end_time_s;
}

WireObject WorkHistory::as_wire() const {
    WireObject airtime;
    for (std::size_t side = 0; side < kSides.size(); ++side) {
        WireObject buckets;
        for (std::size_t threshold = 0; threshold < kThresholdNames.size(); ++threshold)
            buckets.emplace_back(kThresholdNames[threshold], Wire(airtime_s[side][threshold]));
        airtime.emplace_back(kSides[side], Wire(std::move(buckets)));
    }
    return {{"work_j", Wire(work_j)},
            {"last_id", last_id.has_value() ? Wire(*last_id) : Wire(nullptr)},
            {"last_end", last_end.has_value() ? Wire(*last_end) : Wire(nullptr)},
            {"airtime_s", Wire(std::move(airtime))}, {"duration_s", Wire(duration_s)}};
}

WorkHistory WorkHistory::from_wire(const WireObject &value) {
    constexpr std::array<std::string_view, 5> keys{
        "work_j", "last_id", "last_end", "airtime_s", "duration_s"};
    exact(value, keys, "state.runtime.history");
    WorkHistory result;
    result.work_j = object(required(value, "work_j"));
    for (const auto &[name, work] : result.work_j)
        finite(number(work), "state.runtime.history.work_j." + name);
    if (!is_none(required(value, "last_id"))) {
        result.last_id = integer(required(value, "last_id"));
        if (*result.last_id < 0)
            throw std::invalid_argument("state.runtime.history.last_id must be nonnegative");
    }
    if (!is_none(required(value, "last_end"))) {
        result.last_end = number(required(value, "last_end"));
        finite(*result.last_end, "state.runtime.history.last_end", true);
    }
    if (result.last_id.has_value() != result.last_end.has_value())
        throw std::invalid_argument("state.runtime.history interval markers must agree");
    result.duration_s = number(required(value, "duration_s"));
    finite(result.duration_s, "state.runtime.history.duration_s", true);
    const auto &airtime = object(required(value, "airtime_s"));
    exact(airtime, kSides, "state.runtime.history.airtime_s");
    for (std::size_t side = 0; side < kSides.size(); ++side) {
        const auto &buckets = object(required(airtime, kSides[side]));
        exact(buckets, kThresholdNames, "state.runtime.history.airtime_s.buckets");
        for (std::size_t threshold = 0; threshold < kThresholdNames.size(); ++threshold) {
            result.airtime_s[side][threshold] = number(required(buckets, kThresholdNames[threshold]));
            finite(result.airtime_s[side][threshold], "state.runtime.history.airtime_s", true);
        }
    }
    return result;
}

} // namespace runtime
