#include "preview_csv.hpp"
#include "../geometry/specs.hpp"
#include <algorithm>
#include <cmath>
#include <exception>
#include <ostream>
#include <stdexcept>
#include <type_traits>

namespace bike::research {
namespace {
void validate_options(double interval, int flush) {
    if (!std::isfinite(interval) || interval <= 0.) throw std::invalid_argument("preview csv interval must be positive and finite");
    if (flush < 1) throw std::invalid_argument("preview csv flush_every must be positive");
}
void validate_clock(double time, std::int64_t generation) {
    if (!std::isfinite(time) || time < 0. || generation < 0)
        throw std::invalid_argument("preview requires finite nonnegative simulated time and generation");
}
std::string scalar_text(const runtime::Wire& wire) {
    return std::visit([](const auto& value) -> std::string {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, std::monostate>) return {};
        else if constexpr (std::is_same_v<T, bool>) return value ? "True" : "False";
        else if constexpr (std::is_same_v<T, std::int64_t>) return std::to_string(value);
        else if constexpr (std::is_same_v<T, double>) return geometry::detail::py_repr(value);
        else if constexpr (std::is_same_v<T, std::string>) return value;
        else throw std::invalid_argument("preview row fields must be scalars");
    }, wire.value);
}
std::string quote(std::string_view value) {
    if (value.find_first_of(",\"\r\n") == std::string_view::npos) return std::string(value);
    std::string result = "\"";
    for (const char ch : value) { if (ch == '"') result += '"'; result += ch; }
    return result + '"';
}
std::string line(const std::vector<std::string>& values) {
    std::string result;
    bool first = true;
    for (const auto& value : values) {
        if (!first) result += ',';
        first = false; result += quote(value);
    }
    return result + "\r\n";
}
} // namespace

PhysicalLiveCsv::PhysicalLiveCsv(const std::filesystem::path& path, double interval, int flush)
    : interval_s_(interval), flush_every_(flush) {
    validate_options(interval, flush);
    if (path.empty()) throw std::invalid_argument("preview path must be nonempty");
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    file_ = std::make_unique<std::ofstream>();
    file_->exceptions(std::ios::failbit | std::ios::badbit);
    file_->open(path, std::ios::out | std::ios::binary | std::ios::trunc);
    stream_ = file_.get();
}
PhysicalLiveCsv::PhysicalLiveCsv(std::ostream& stream, double interval, int flush)
    : stream_(&stream), interval_s_(interval), flush_every_(flush) { validate_options(interval, flush); }
PhysicalLiveCsv::~PhysicalLiveCsv() { try { close(); } catch (...) {} }
void PhysicalLiveCsv::require_open() const {
    if (closed_) throw std::runtime_error("preview csv is closed");
    if (failed_) throw std::runtime_error("preview csv write failed; close the stream before proceeding");
}
bool PhysicalLiveCsv::due(double time, std::int64_t generation) const {
    require_open(); validate_clock(time, generation);
    return generation_ != generation || time + 1e-12 >= next_time_s_;
}
void PhysicalLiveCsv::emit(const runtime::WireObject& row, bool flush) {
    require_open();
    auto columns = columns_;
    if (columns.empty())
        for (const auto& [name, ignored] : row) {
            (void)ignored;
            if (std::find(columns.begin(), columns.end(), name) != columns.end())
                throw std::invalid_argument("preview row contains a duplicate column");
            columns.push_back(name);
        }
    std::vector<std::string> values;
    values.reserve(columns.size());
    for (const auto& name : columns) {
        const auto* value = runtime::sample_wire::find(row, name);
        values.push_back(value ? scalar_text(*value) : std::string{});
    }
    const auto bytes = (columns_.empty() ? line(columns) : std::string{}) + line(values);
    try {
        stream_->write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        if (!*stream_) throw std::ios_base::failure("preview csv write failed");
        if (flush) {
            stream_->flush();
            if (!*stream_) throw std::ios_base::failure("preview csv flush failed");
        }
    } catch (...) { failed_ = true; throw; }
    columns_ = std::move(columns);
}
void PhysicalLiveCsv::write(double time, std::int64_t generation, const runtime::WireObject& row) {
    if (!due(time, generation)) return;
    const bool changed = generation_ != generation;
    const std::string event = changed && generation_ ? "reset" : "";
    double next = changed ? time : next_time_s_;
    while (next <= time + 1e-12) {
        const auto advanced = next + interval_s_;
        if (!std::isfinite(advanced) || advanced <= next)
            throw std::invalid_argument("preview interval cannot advance this simulated clock");
        next = advanced;
    }
    runtime::WireObject record{{"generation", generation}, {"event", event}};
    for (const auto& [key, value] : row) runtime::sample_wire::set(record, key, value);
    const bool flush = !event.empty() || rows_since_flush_ >= flush_every_ - 1;
    emit(record, flush);
    generation_ = generation; next_time_s_ = next;
    rows_since_flush_ = flush ? 0 : rows_since_flush_ + 1;
}
void PhysicalLiveCsv::write_marker(std::string marker) {
    emit(runtime::WireObject{{"generation", generation_ ? runtime::Wire{*generation_} : runtime::Wire{std::string{}}},
        {"event", std::move(marker)}}, true);
    rows_since_flush_ = 0;
}
void PhysicalLiveCsv::close() {
    if (closed_) return;
    std::exception_ptr failure;
    try {
        stream_->flush();
        if (!*stream_) throw std::ios_base::failure("preview csv flush failed");
    } catch (...) { failure = std::current_exception(); }
    try { if (file_) file_->close(); }
    catch (const std::exception& error) {
        if (!failure) failure = std::current_exception();
        else { try { cleanup_notes_.push_back(std::string("preview stream close also failed: ") + error.what()); } catch (...) {} }
    }
    closed_ = true;
    if (failure) std::rethrow_exception(failure);
}
} // namespace bike::research
