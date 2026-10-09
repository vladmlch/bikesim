#include "samples.hpp"

#include <algorithm>
#include <cmath>
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

// NOLINTNEXTLINE(misc-no-recursion) recursion follows an owned, finite Wire tree
void flatten(const Wire &value, const std::string &prefix,
             std::map<std::string, double> &destination) {
    if (const auto *fields = std::get_if<WireObject>(&value.value)) {
        for (const auto &[key, item] : *fields) {
            std::string nested{prefix};
            if (!nested.empty()) nested += '.';
            nested += key;
            flatten(item, nested, destination);
        }
    } else if (const auto *items = std::get_if<WireArray>(&value.value)) {
        for (std::size_t i = 0; i < items->size(); ++i) {
            std::string nested{prefix};
            nested += '.';
            nested += std::to_string(i);
            flatten((*items)[i], nested, destination);
        }
    } else if (const auto *real = std::get_if<double>(&value.value)) {
        destination.emplace(prefix, *real);
    } else if (const auto *whole = std::get_if<std::int64_t>(&value.value)) {
        destination.emplace(prefix, static_cast<double>(*whole));
    } else if (const auto *flag = std::get_if<bool>(&value.value)) {
        destination.emplace(prefix, *flag ? 1. : 0.);
    }
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

std::map<std::string, double>
PhysicalSampleData::flat_columns(const ColumnLayout &layout) const {
    std::map<std::string, double> result;
    flatten(Wire(as_wire()), "", result);
    result["x_m"] = qpos.at(layout.root_x_qpos);
    result["speed_mps"] = qvel.at(layout.root_x_dof);
    result["pitch_rad"] = qpos.at(layout.root_pitch_qpos);
    result["interval_start_s"] = time_s;
    result["interval_dt_s"] = dt_s();
    for (std::size_t i = 0; i < qpos.size(); ++i)
        result["prestep_qpos_" + std::to_string(i)] = qpos[i];
    for (std::size_t i = 0; i < qvel.size(); ++i)
        result["prestep_qvel_" + std::to_string(i)] = qvel[i];
    for (const auto &[name, power] : powers_w) {
        result["power_" + name + "_w"] = number(power);
        result[name + "_power_w"] = number(power);
    }
    if (const auto *work = find(channels, "component_work_j"))
        for (const auto &[name, value] : object(*work))
            result[name + "_work_j"] = number(value);
    const auto &mass = object(required(channels, "mass"));
    result["compiled_mass_kg"] = number(required(mass, "mass_kg"));
    const auto &com = array(required(mass, "com_m"));
    constexpr std::array<std::string_view, 3> axes{"x", "y", "z"};
    for (std::size_t i = 0; i < axes.size(); ++i)
        result["compiled_com_" + std::string(axes[i]) + "_m"] = number(com.at(i));
    for (const auto name : {"kinetic_energy_j", "gravitational_energy_j"})
        result[name] = number(required(mass, name));
    const auto &energy = object(required(channels, "energy"));
    result["mechanical_energy_j"] = number(required(energy, "mechanical_energy_j"));
    result["energy_residual_j"] = number(required(energy, "residual_j"));
    return result;
}

SampleColumns make_columns(std::span<const SamplePtr> samples,
                           const ColumnLayout &layout) {
    SampleColumns result;
    const double missing = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const auto flat = samples[i]->flat_columns(layout);
        for (const auto &[key, value] : flat) {
            auto [slot, inserted] = result.try_emplace(key);
            if (inserted) slot->second.assign(samples.size(), missing);
            slot->second[i] = value;
        }
    }
    return result;
}

SampleColumns join_columns(std::span<const RecordedBlock> blocks) {
    std::size_t total = 0;
    for (const auto &block : blocks) {
        if (block.rows > std::numeric_limits<std::size_t>::max() - total)
            throw std::overflow_error("recorded column row count overflow");
        total += block.rows;
    }
    SampleColumns result;
    std::size_t offset = 0;
    for (const auto &block : blocks) {
        for (const auto &[name, values] : block.columns) {
            if (values.size() != block.rows)
                throw std::logic_error("recorded column block width mismatch");
            auto [slot, inserted] = result.try_emplace(name);
            if (inserted)
                slot->second.assign(total, std::numeric_limits<double>::quiet_NaN());
            const auto destination =
                std::span<double>(slot->second).subspan(offset, block.rows);
            std::ranges::copy(values, destination.begin());
        }
        offset += block.rows;
    }
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
