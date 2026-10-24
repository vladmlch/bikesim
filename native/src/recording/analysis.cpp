#include "reader.hpp"
#include "analysis_detail.hpp"
#include <array>
#include <map>
#include <set>
#include <format>

namespace bike::recording {
namespace {
using namespace detail;
constexpr std::array defaults{"physics.time_s", "physics.interval_end_s", "physics.qpos", "physics.qvel",
    "channel_energy.residual_j", "channel_model_status.model_status"};
constexpr std::array provenance{"channel_energy.measured", "channel_energy.source_interval_id",
    "channel_energy.source_time_s", "channel_mass.measurement_scope"};

std::int64_t scalar_count(const Recording& recording, std::string_view sql, std::span<const Value> params = {}) {
    const auto rows = recording.rows(sql, params);
    if (rows.size() != 1 || rows.front().size() != 1) throw RecordingError("invalid scalar count result");
    return count(rows.front().front().second);
}
std::vector<std::string> requested_fields(const std::optional<std::string>& requested) {
    std::vector<std::string> out;
    if (!requested) { for (const auto* field_name : defaults) out.emplace_back(field_name); return out; }
    std::size_t start = 0;
    for (;;) {
        const auto end = requested->find(',', start);
        auto const name = requested->substr(start, end == std::string::npos ? end : end - start);
        const auto first = name.find_first_not_of(" \t\n\r"), last = name.find_last_not_of(" \t\n\r");
        if (first == std::string::npos) throw std::invalid_argument("fields must contain nonempty catalog names");
        out.push_back(name.substr(first, last - first + 1));
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return out;
}
struct Selection {
    std::string label, canonical, root, expression;
    Value index, parameter;
    Object definition;
};
std::vector<Selection> selection(const Recording& reader, std::int64_t episode_id,
    const std::vector<std::string>& requested, bool companions = true) {
    std::map<std::string, Object> catalog;
    for (auto row : reader.rows("SELECT * FROM field_catalog")) {
        const auto name = string(field(row, "table_name")) + '.' + string(field(row, "field_name"));
        if (!catalog.emplace(name, std::move(row)).second) throw RecordingError("duplicate field catalog identity");
    }
    const std::array params{integer(episode_id)};
    const auto aliases = reader.rows("SELECT * FROM alias_catalog WHERE episode_id=? ORDER BY alias", params);
    std::vector<Selection> out; std::set<std::string> labels;
    for (const auto& label : requested) {
        if (label.empty()) throw std::invalid_argument("fields must contain nonempty catalog names");
        if (labels.contains(label)) continue;
        Selection chosen{.label=label, .canonical=label, .root={}, .expression={}, .index={}, .parameter={}, .definition={}};
        if (label.starts_with("spatial.")) {
            if (reader.schema_version() == 1) throw std::invalid_argument("recording schema 1 has no spatial channels");
            chosen.canonical = "channel_spatial." + label.substr(8);
            if(label=="spatial.hold_work_j"||label=="spatial.tyre_loss_j")chosen.canonical="channel_spatial_energy."+label.substr(8);
            for(const auto side:{"front","rear"}) {
                const std::string base=std::string("spatial.")+side+"_wheel_";
                if(label==base+"position_world_m")chosen.canonical="channel_spatial_geometry."+std::string(side)+"_wheel_position_world_m";
                constexpr std::array axes{"x","y","z"};
                for(std::size_t axis=0;axis<axes.size();++axis)if(label==base+axes[axis]+"_m") {
                    chosen.canonical="channel_spatial_geometry."+std::string(side)+"_wheel_position_world_m";
                    chosen.index=integer(static_cast<std::int64_t>(axis));
                }
            }
        }
        if (!catalog.contains(chosen.canonical)) {
            for (const auto& alias : aliases) {
                const auto name = string(field(alias, "alias"));
                bool matches = name == label; std::optional<std::string> component;
                if (!matches) {
                    const auto marker = name.find("{component}");
                    if (marker != std::string::npos) {
                        const auto prefix = name.substr(0, marker), suffix = name.substr(marker + 11);
                        if (label.starts_with(prefix) && label.ends_with(suffix) && label.size() > prefix.size() + suffix.size()) {
                            component = label.substr(prefix.size(), label.size() - prefix.size() - suffix.size()); matches = true;
                        }
                    }
                }
                if (matches) {
                    chosen.canonical = string(field(alias, "canonical"));
                    chosen.index = field(alias, "vector_index");
                    chosen.parameter = component ? text(*component) : field(alias, "parameter");
                    break;
                }
            }
        }
        for (const auto& [root, ignored] : catalog) {
            (void)ignored;
            if ((chosen.canonical == root || chosen.canonical.starts_with(root + '.')) && root.size() > chosen.root.size()) chosen.root = root;
        }
        if (chosen.root.empty()) throw std::invalid_argument("unknown recording field: " + label);
        chosen.definition = catalog.at(chosen.root);
        const auto table = string(field(chosen.definition, "table_name")), name = string(field(chosen.definition, "field_name"));
        if (table == "forces") {
            if (name != "qfrc" && name != "power_w")
                throw std::invalid_argument("select forces.qfrc or forces.power_w; use sql for component identities");
            if (io::is_null(chosen.parameter))
                chosen.expression = "(SELECT map(list(f.component ORDER BY f.component),list(f." + quote(name) +
                    " ORDER BY f.component)) FROM forces f WHERE f.episode_id=telemetry.episode_id AND f.interval_id=telemetry.interval_id)";
            else chosen.expression = "(SELECT f." + quote(name) + " FROM forces f WHERE f.episode_id=telemetry.episode_id "
                "AND f.interval_id=telemetry.interval_id AND f.component=" + literal(string(chosen.parameter)) + ')';
        } else if (table == "physics" || table.starts_with("channel_")) chosen.expression = quote(chosen.root);
        else throw std::invalid_argument(std::format("{} is not an interval field; query {} using sql", label, table));
        const auto definition = io::parse_recording_json(string(field(chosen.definition, "definition_json")));
        auto current = definition;
        auto remainder = chosen.canonical.substr(chosen.root.size());
        if (!remainder.empty()) {
            remainder.erase(0, 1);
            std::size_t begin = 0;
            for (;;) {
                const auto end = remainder.find('.', begin);
                const auto member = remainder.substr(begin, end == std::string::npos ? end : end - begin);
                const auto dtype = string(field(current, "dtype"));
                if (dtype.starts_with("map<")) set(current, "dtype", text(dtype.substr(4, dtype.size() - 5)));
                else {
                    bool found = false;
                    if (const auto* children = find(current, "children"))
                        for (const auto& child : io::as_array(*children, "field children"))
                            if (string(field(child, "key")) == member) { current = child; found = true; break; }
                    if (!found) throw std::invalid_argument("unknown structured recording field: " + chosen.canonical);
                }
                chosen.expression = '(' + chosen.expression + ")[" + literal(member) + ']';
                if (end == std::string::npos) break;
                begin = end + 1;
            }
        } else if (!io::is_null(chosen.parameter) && table != "forces") {
            if (!string(field(definition, "dtype")).starts_with("map<")) throw std::invalid_argument(label + " is not a map member");
            chosen.expression = '(' + chosen.expression + ")[" + literal(string(chosen.parameter)) + ']';
        }
        if (!io::is_null(chosen.index)) {
            const auto index = count(chosen.index);
            if (index < 0 || index == std::numeric_limits<std::int64_t>::max()) throw RecordingError("invalid catalog vector index");
            chosen.expression = '(' + chosen.expression + ")[" + std::to_string(index + 1) + ']';
        }
        out.push_back(chosen); labels.insert(label);
        if (companions) {
            std::vector<std::string> masks{chosen.root + "_present"};
            if (const auto* path = find(definition, "path")) {
                const auto& parts = io::as_array(*path, "field path");
                std::string parent;
                for (std::size_t i = 0; i < parts.size(); ++i) {
                    if (i) parent += '.';
                    parent += string(parts[i]);
                    if (i + 1 < parts.size() || i == 0)
                        for (auto suffix : {"_present", "_valid"}) masks.push_back(std::format("{}.__{}{}", table, parent, suffix));
                }
            }
            for (const auto& mask : masks)
                if (catalog.contains(mask) && labels.insert(mask).second)
                    out.push_back(Selection{.label=mask, .canonical=mask, .root=mask, .expression=quote(mask), .index={}, .parameter={}, .definition=catalog.at(mask)});
        }
    }
    return out;
}
std::string projection(const std::vector<Selection>& selected) {
    std::string out = "episode_id, interval_id";
    for (const auto& field : selected) out += ", " + field.expression + " AS " + quote(field.label);
    return out;
}
Value summary(const Recording& reader, std::int64_t episode_id, std::uint64_t maximum) {
    const auto episode = reader.episode(episode_id);
    const auto saved = io::parse_recording_json(string(field(episode, "summary_json")));
    const std::array params{integer(episode_id), integer(static_cast<std::int64_t>(maximum + 1))};
    auto groups = reader.rows("SELECT e.kind,count(*) AS count,min(p.time_s) AS first_time_s,max(p.time_s) AS last_time_s "
        "FROM events e LEFT JOIN physics p ON e.episode_id=p.episode_id AND e.processed_step=p.interval_id "
        "WHERE e.episode_id=? GROUP BY e.kind ORDER BY e.kind LIMIT ?", params);
    const auto windows_limit = std::min<std::uint64_t>(maximum, 20);
    const std::array anchor_params{integer(episode_id), integer(static_cast<std::int64_t>(windows_limit + 1))};
    const auto anchors = reader.rows("SELECT e.event_id,e.kind,p.time_s,p.dt_s FROM events e JOIN physics p "
        "ON e.episode_id=p.episode_id AND e.processed_step=p.interval_id WHERE e.episode_id=? "
        "AND e.kind IN ('model_first_failure','derived_transition') ORDER BY e.event_id LIMIT ?", anchor_params);
    std::vector<Object> windows;
    for (std::size_t i = 0; i < std::min<std::uint64_t>(anchors.size(), windows_limit); ++i) {
        const auto& anchor = anchors[i];
        const auto time = io::as_double(field(anchor, "time_s"), "event time"), dt = io::as_double(field(anchor, "dt_s"), "event dt");
        windows.push_back(Object{{"event_id", field(anchor, "event_id")}, {"kind", field(anchor, "kind")},
            {"data", window(reader, episode_id, time - dt, time + 2 * dt, {}, std::min<std::uint64_t>(maximum, 10))}});
    }
    return Value{Object{{"outcome", maybe(saved, "outcome")}, {"model_status", maybe(saved, "model_status")},
        {"first_failure", maybe(saved, "first_failure")}, {"metrics", find(saved, "episode_metrics") ? field(saved, "episode_metrics") : Value{Object{}}},
        {"resolved_config", maybe(saved, "resolved_config")}, {"physical_rows", field(episode, "physical_rows")},
        {"complete_control_steps", field(episode, "complete_control_steps")}, {"replay_eligible", field(episode, "replay_eligible")},
        {"events", bounded(std::move(groups), maximum)}, {"event_windows", bounded(std::move(windows), windows_limit)},
        {"event_windows_truncated", Value{anchors.size() > windows_limit}}}};
}
struct Statistics {
    double absolute = 0., relative = 0.;
    std::int64_t nonfinite = 0, matching_invalid = 0, signed_zero = 0;
};
std::optional<double> floating(const Value& value) {
    if (const auto* number = std::get_if<io::Number>(&value.data); number && number->lexeme.find_first_of(".eE") != std::string::npos)
        return io::as_double(value, "comparison float");
    if (const auto* tag = std::get_if<io::Tagged>(&value.data); tag && tag->kind == "float64")
        for (const auto& [key, token] : tag->fields) if (key == "value") {
            if (token == "nan") return std::numeric_limits<double>::quiet_NaN();
            if (token == "+inf") return std::numeric_limits<double>::infinity();
            if (token == "-inf") return -std::numeric_limits<double>::infinity();
        }
    return {};
}
// NOLINTNEXTLINE(misc-no-recursion) recursion mirrors the Python reference structure
Value difference(const Value& left, const Value& right, const std::string& path, double atol, double rtol, Statistics& stats) {
    const auto mismatch = [&](std::string_view kind) { return Value{Object{{"path", text(path)}, {"kind", text(kind)}, {"left", left}, {"right", right}}}; };
    if (io::is_null(left) || io::is_null(right)) return io::is_null(left) && io::is_null(right) ? Value{} : mismatch("null_or_presence");
    if (const auto* a = std::get_if<Object>(&left.data)) {
        if (const auto* b = std::get_if<Object>(&right.data)) {
            std::set<std::string> names;
            for (const auto& [key, ignored] : *a) { (void)ignored; names.insert(key); }
            for (const auto& [key, ignored] : *b) { (void)ignored; names.insert(key); }
            Value first;
            for (const auto& key : names) {
                const auto* av = find(*a, key); const auto* bv = find(*b, key);
                Value current;
                if (!av || !bv) current = Value{Object{{"path", text(std::format("{}.{}", path, key))}, {"kind", text("presence")},
                    {"left_present", Value{av != nullptr}}, {"right_present", Value{bv != nullptr}}}};
                else current = difference(*av, *bv, std::format("{}.{}", path, key), atol, rtol, stats);
                if (io::is_null(first) && !io::is_null(current)) first = std::move(current);
            }
            return first;
        }
    }
    if (const auto* a = std::get_if<Array>(&left.data)) {
        if (const auto* b = std::get_if<Array>(&right.data)) {
            if (a->size() != b->size()) return Value{Object{{"path", text(path)}, {"kind", text("vector_width")},
                {"left", integer(static_cast<std::int64_t>(a->size()))}, {"right", integer(static_cast<std::int64_t>(b->size()))}}};
            Value first;
            for (std::size_t i = 0; i < a->size(); ++i) {
                auto current = difference((*a)[i], (*b)[i], path + '[' + std::to_string(i) + ']', atol, rtol, stats);
                if (io::is_null(first) && !io::is_null(current)) first = std::move(current);
            }
            return first;
        }
    }
    const auto a = floating(left), b = floating(right);
    if (a && b) {
        if (!std::isfinite(*a) || !std::isfinite(*b)) {
            ++stats.nonfinite;
            if ((std::isnan(*a) && std::isnan(*b)) || *a == *b) { ++stats.matching_invalid; return Value{}; }
            return mismatch("nonfinite");
        }
        const auto absolute = std::abs(*a - *b), scale = std::max(std::abs(*a), std::abs(*b));
        const auto relative = scale != 0. ? absolute / scale : 0.;
        stats.absolute = std::max(stats.absolute, absolute); stats.relative = std::max(stats.relative, relative);
        if (*a == 0. && *b == 0. && std::signbit(*a) != std::signbit(*b)) ++stats.signed_zero;
        if (absolute <= atol + rtol * scale) return Value{};
        auto result = mismatch("float"); set(result, "absolute_error", real(absolute)); set(result, "relative_error", real(relative)); return result;
    }
    if (a.has_value() != b.has_value() || left.data.index() != right.data.index()) return mismatch("type");
    return equal(left, right) ? Value{} : mismatch("category");
}
Array compare_field_schema(const Recording& left, const Recording& right, std::int64_t left_episode, std::int64_t right_episode,
    const std::vector<Selection>& lf, const std::vector<Selection>& rf) {
    Array incompatible;
    for (std::size_t i = 0; i < std::min(lf.size(), rf.size()); ++i) {
        const auto& a = lf[i]; const auto& b = rf[i];
        bool same = a.canonical == b.canonical && equal(a.index, b.index) && equal(a.parameter, b.parameter);
        for (auto key : {"dtype", "unit", "phase", "measurement", "frame", "source", "definition_json"}) same = same && equal(field(a.definition, key), field(b.definition, key));
        if (!same) incompatible.push_back(Value{Object{{"field", text(a.label)}, {"reason", text("field_semantics")}}});
    }
    if (lf.size() != rf.size()) incompatible.push_back(Value{Object{{"reason", text("field_or_presence_schema")}}});
    const std::array lparams{integer(left_episode)}, rparams{integer(right_episode)};
    constexpr auto coordinates = "SELECT kind,coordinate_index,joint,component,dof_kind,unit FROM coordinate_catalog WHERE episode_id=? ORDER BY kind,coordinate_index";
    if (!equal(Value{values(left.rows(coordinates, lparams))}, Value{values(right.rows(coordinates, rparams))}))
        incompatible.push_back(Value{Object{{"reason", text("coordinate_identity")}}});
    return incompatible;
}
void compare_streams(const Recording& left, const Recording& right, std::int64_t left_episode, std::int64_t right_episode,
    const std::vector<Selection>& lf, const std::vector<Selection>& rf, const Value& lmeta, const Value& rmeta,
    // NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    double atol, double rtol, std::uint64_t maximum, Array incompatible, Value& result) {
    const std::array lparams{integer(left_episode)}, rparams{integer(right_episode)};
    constexpr auto clock = R"(, "physics.time_s" AS __start, "physics.interval_end_s" AS __end, "physics.dt_s" AS __dt)";
    // Independent readers make comparing a recording to itself safe: a new
    // streaming query never invalidates the other side's engine cursor.
    auto left_stream = open_recording(left.path()), right_stream = open_recording(right.path());
    auto lc = left_stream->stream("SELECT " + projection(lf) + clock + " FROM telemetry WHERE episode_id=? ORDER BY interval_id", lparams);
    auto rc = right_stream->stream("SELECT " + projection(rf) + clock + " FROM telemetry WHERE episode_id=? ORDER BY interval_id", rparams);
    std::vector<Statistics> stats(lf.size()); std::int64_t compared = 0, mismatched = 0;
    Value first, unmatched; bool stop = false;
    while (!stop) {
        const auto a = lc.next_batch(1), b = rc.next_batch(1);
        if (a.empty() && b.empty()) break;
        if (a.empty() || b.empty()) {
            unmatched = Value{Object{{"left_first_interval", a.empty() ? Value{} : field(a[0], "interval_id")},
                {"right_first_interval", b.empty() ? Value{} : field(b[0], "interval_id")}}}; break;
        }
        for (auto key : {"interval_id", "__start", "__end", "__dt"})
            if (!equal(field(a[0], key), field(b[0], key))) {
                incompatible.push_back(Value{Object{{"reason", text("time_grid")}, {"left_interval", field(a[0], "interval_id")},
                    {"right_interval", field(b[0], "interval_id")}, {"left_time_s", field(a[0], "__start")}, {"right_time_s", field(b[0], "__start")}}}); stop = true; break;
            }
        if (stop) break;
        for (auto key : provenance)
            if (!equal(field(a[0], key), field(b[0], key))) {
                incompatible.push_back(Value{Object{{"reason", text("measurement_provenance")}, {"interval_id", field(a[0], "interval_id")}}}); stop = true; break;
            }
        if (stop) break;
        ++compared;
        for (std::size_t i = 0; i < lf.size(); ++i) {
            auto diff = difference(field(a[0], lf[i].label), field(b[0], rf[i].label), lf[i].label, atol, rtol, stats[i]);
            if (!io::is_null(diff)) {
                ++mismatched;
                if (io::is_null(first)) { first = std::move(diff); set(first, "interval_id", field(a[0], "interval_id")); set(first, "time_s", field(a[0], "__start")); }
            }
        }
    }
    std::vector<Object> deviations; std::int64_t invalid_pairs = 0;
    for (std::size_t i = 0; i < lf.size(); ++i) {
        const auto& value = stats[i]; invalid_pairs += value.matching_invalid;
        deviations.push_back(Object{{"field", text(lf[i].label)}, {"max_absolute_error", real(value.absolute)}, {"max_relative_error", real(value.relative)},
            {"nonfinite_pairs", integer(value.nonfinite)}, {"matching_invalid_pairs", integer(value.matching_invalid)}, {"signed_zero_differences", integer(value.signed_zero)}});
    }
    set(result, "compatible", Value{incompatible.empty()}); set(result, "agrees", Value{incompatible.empty() && io::is_null(first) && io::is_null(unmatched)});
    set(result, "first_mismatch", std::move(first)); set(result, "mismatched_field_rows", integer(mismatched)); set(result, "compared_intervals", integer(compared));
    set(result, "incompatibilities", Value{std::move(incompatible)});
    set(result, "unmatched_extents", Value{Object{{"left_intervals", integer(count(field(lmeta, "physical_rows")) - compared)},
        {"right_intervals", integer(count(field(rmeta, "physical_rows")) - compared)}, {"first", std::move(unmatched)}}});
    set(result, "deviations", bounded(std::move(deviations), maximum, static_cast<std::int64_t>(lf.size())));
    set(result, "matching_invalid_pairs", integer(invalid_pairs));
    set(result, "interpretation", text(invalid_pairs ? "Matching nonfinite values are agreeing INVALID evidence, not a valid physical result." :
        "Agreement concerns saved fields, not physical calibration."));
}
} // namespace

io::Value overview(const Recording& reader, std::uint64_t maximum) {
    limit(maximum);
    auto run = reader.rows("SELECT * FROM runs").at(0);
    set(run, "metadata", io::parse_recording_json(string(pop(run, "metadata_json"))));
    const std::array params{integer(static_cast<std::int64_t>(maximum + 1))};
    auto episodes = reader.rows("SELECT e.*,min(p.time_s) AS first_time_s,max(p.interval_end_s) AS last_time_s FROM episodes e "
        "LEFT JOIN physics p USING(episode_id) GROUP BY ALL ORDER BY e.episode_id LIMIT ?", params);
    for (auto& row : episodes) set(row, "summary", io::parse_recording_json(string(pop(row, "summary_json"))));
    return Value{Object{{"run", Value{std::move(run)}}, {"manifest", reader.manifest()},
        {"episodes", bounded(std::move(episodes), maximum, scalar_count(reader, "SELECT count(*) FROM episodes"))},
        {"limitations", Value{Array{text("Full launch retained in RAM; no crash-recovery promise."),
            text("Ordinary recipes are not checked research replay."), text("Model/numerical validity is not calibration evidence.")}}}}};
}
io::Value fields(const Recording& reader, std::optional<std::string> pattern, std::uint64_t maximum) {
    limit(maximum); std::string condition; std::vector<Value> params;
    if (pattern) {
        std::string like;
        for (char const c : *pattern) { if (c == '\\' || c == '%' || c == '_') like += '\\'; like += c == '*' ? '%' : c; }
        if (pattern->find('*') == std::string::npos) like = '%' + like + '%';
        condition = " WHERE (table_name || '.' || field_name) LIKE ? ESCAPE '\\'"; params.push_back(text(like));
    }
    const auto total = scalar_count(reader, "SELECT count(*) FROM field_catalog" + condition, params);
    params.push_back(integer(static_cast<std::int64_t>(maximum + 1)));
    auto rows = reader.rows("SELECT * FROM field_catalog" + condition + " ORDER BY table_name,field_name LIMIT ?", params);
    for (auto& row : rows) {
        set(row, "canonical", text(string(field(row, "table_name")) + '.' + string(field(row, "field_name"))));
        set(row, "definition", io::parse_recording_json(string(pop(row, "definition_json"))));
    }
    return bounded(std::move(rows), maximum, total);
}
io::Value events(const Recording& reader, std::int64_t episode_id, std::optional<std::string> kind, std::uint64_t maximum) {
    limit(maximum); (void)reader.episode(episode_id);
    std::vector<Value> params{integer(episode_id)}; std::string condition = "e.episode_id=?";
    if (kind) { condition += " AND e.kind=?"; params.push_back(text(*kind)); }
    const auto total = scalar_count(reader, "SELECT count(*) FROM events e WHERE " + condition, params);
    params.push_back(integer(static_cast<std::int64_t>(maximum + 1)));
    auto rows = reader.rows("SELECT e.*,p.time_s AS interval_time_s FROM events e LEFT JOIN physics p "
        "ON p.episode_id=e.episode_id AND p.interval_id=e.processed_step WHERE " + condition + " ORDER BY e.event_id LIMIT ?", params);
    for (auto& row : rows) set(row, "values", io::parse_recording_json(string(pop(row, "values_json"))));
    auto result = bounded(std::move(rows), maximum, total); set(result, "episode_id", integer(episode_id)); return result;
}
io::Value window(const Recording& reader, std::int64_t episode_id, double start, double end,
    const std::optional<std::string>& requested, std::uint64_t maximum) {
    limit(maximum); (void)reader.episode(episode_id);
    if (!std::isfinite(start) || !std::isfinite(end) || end <= start) throw std::invalid_argument("window requires finite start < end");
    const auto selected = selection(reader, episode_id, requested_fields(requested));
    const std::string condition = R"(episode_id=? AND "physics.time_s">=? AND "physics.time_s"<?)";
    const std::array count_params{integer(episode_id), real(start), real(end)};
    const auto total = scalar_count(reader, "SELECT count(*) FROM telemetry WHERE " + condition, count_params);
    const std::array params{real(end), integer(episode_id), real(start), real(end), integer(static_cast<std::int64_t>(maximum + 1))};
    auto rows = reader.rows("SELECT " + projection(selected) + ", \"physics.interval_end_s\">? AS overlaps_end FROM telemetry WHERE " +
        condition + " ORDER BY interval_id LIMIT ?", params);
    const std::array overlap_params{integer(episode_id), real(start), real(start), integer(static_cast<std::int64_t>(maximum + 1))};
    auto overlaps = reader.rows("SELECT episode_id,interval_id,time_s,interval_end_s FROM physics "
        "WHERE episode_id=? AND time_s<? AND interval_end_s>? ORDER BY interval_id LIMIT ?", overlap_params);
    auto result = bounded(std::move(rows), maximum, total); Array selected_fields;
    for (const auto& chosen : selected) selected_fields.push_back(Value{Object{{"name", text(chosen.label)}, {"canonical", text(chosen.canonical)}}});
    set(result, "episode_id", integer(episode_id)); set(result, "start_s", real(start)); set(result, "end_s", real(end));
    set(result, "selection", text("interval start in [start_s,end_s); no interpolation"));
    set(result, "fields", Value{std::move(selected_fields)}); set(result, "overlap_before_start", bounded(std::move(overlaps), maximum)); return result;
}

io::Value compare(const Recording& left, const Recording& right, std::int64_t left_episode, std::int64_t right_episode,
    std::string_view mode, const std::optional<std::string>& requested, double atol, double rtol, std::uint64_t maximum) {
    limit(maximum);
    if (!std::isfinite(atol) || atol < 0. || !std::isfinite(rtol) || rtol < 0.) throw std::invalid_argument("atol and rtol must be finite and nonnegative");
    const auto lmeta = left.episode(left_episode), rmeta = right.episode(right_episode);
    Value result{Object{{"mode", text(mode)}, {"left_episode", integer(left_episode)}, {"right_episode", integer(right_episode)},
        {"atol", real(atol)}, {"rtol", real(rtol)}, {"tolerance_rule", text("abs(a-b) <= atol + rtol*max(abs(a),abs(b))")}, {"alignment", text("none")}}};
    if (mode == "summary") {
        set(result, "left", summary(left, left_episode, maximum)); set(result, "right", summary(right, right_episode, maximum));
        set(result, "interpretation", text("Compare outcome/metrics/event times and saved surrounding windows; no automatic alignment.")); return result;
    }
    if (mode != "exact") throw std::invalid_argument("comparison mode must be exact or summary");
    auto names = requested_fields(requested); for (const auto* name : provenance) names.emplace_back(name);
    const auto lf = selection(left, left_episode, names), rf = selection(right, right_episode, names);
    auto incompatible = compare_field_schema(left, right, left_episode, right_episode, lf, rf);
    const Value validity{Object{{"left", maybe(io::parse_recording_json(string(field(lmeta, "summary_json"))), "model_status")},
        {"right", maybe(io::parse_recording_json(string(field(rmeta, "summary_json"))), "model_status")}}};
    set(result, "validity", validity);
    if (!incompatible.empty()) {
        set(result, "compatible", Value{false}); set(result, "agrees", Value{false}); set(result, "incompatibilities", Value{std::move(incompatible)});
        set(result, "compared_intervals", integer(0)); return result;
    }
    compare_streams(left, right, left_episode, right_episode, lf, rf, lmeta, rmeta, atol, rtol, maximum, std::move(incompatible), result);
    return result;
}
} // namespace bike::recording
