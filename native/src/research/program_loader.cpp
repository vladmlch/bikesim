#include "environment.hpp"
#include "../configuration/load.hpp"
#include "../runtime/setup_values.hpp"
#include <algorithm>
#include <ranges>
#include <stdexcept>

namespace bike::research {
namespace {
using namespace runtime::setupvalue;
void keys(const Value& value, std::initializer_list<std::string_view> allowed, std::string_view path) {
    for (const auto& [name, ignored] : io::as_object(value, path)) {
        (void)ignored;
        if (std::ranges::find(allowed, name) == allowed.end())
            throw std::invalid_argument(std::string(path) + ": unknown field '" + name + "'");
    }
}
::rider::RiderPosture posture(const Value& value) {
    keys(value, {"torso_lean_rad", "pelvis_pitch_rad", "pelvis_offset_m", "use_saddle"}, "posture");
    ::rider::RiderPosture out;
    if (const auto* x = find(value, "torso_lean_rad")) out.torso_lean_rad = io::as_double(*x, "posture.torso_lean_rad");
    if (const auto* x = find(value, "pelvis_pitch_rad")) out.pelvis_pitch_rad = io::as_double(*x, "posture.pelvis_pitch_rad");
    if (const auto* x = find(value, "use_saddle")) out.use_saddle = io::as_bool(*x, "posture.use_saddle");
    if (const auto* x = find(value, "pelvis_offset_m"); x && !std::holds_alternative<std::monostate>(x->data)) {
        const auto& values = io::as_array(*x, "posture.pelvis_offset_m");
        if (values.size() != 2) throw std::invalid_argument("posture.pelvis_offset_m must have width 2");
        out.pelvis_offset_m = std::array<double, 2>{io::as_double(values[0], "posture.pelvis_offset_m"),
                                                 io::as_double(values[1], "posture.pelvis_offset_m")};
    }
    configuration::detail::post_init(out);
    return out;
}
} // namespace

runtime::ResearchPrograms programs_from_documents(const io::Value& rider_doc, const io::Value& demand_doc) {
    runtime::ResearchPrograms out;
    if (!std::holds_alternative<std::monostate>(rider_doc.data)) {
        keys(rider_doc, {"keyframes", "reaction_delay_s"}, "rider program");
        if (const auto* delay = find(rider_doc, "reaction_delay_s")) out.reaction_delay_s = io::as_double(*delay, "rider reaction delay");
        for (const auto& row : io::as_array(field(rider_doc, "keyframes"), "rider keyframes")) {
            keys(row, {"time_s", "posture", "human_torque_nm"}, "rider keyframe");
            runtime::RiderKeyframe frame;
            frame.time_s = io::as_double(field(row, "time_s"), "rider keyframe time");
            if (const auto* pose = find(row, "posture")) frame.posture = posture(*pose);
            if (const auto* effort = find(row, "human_torque_nm"); effort && !std::holds_alternative<std::monostate>(effort->data))
                frame.human_torque_nm = io::as_double(*effort, "rider effort");
            out.rider_frames.push_back(frame);
        }
        if (out.rider_frames.empty()) throw std::invalid_argument("rider program requires keyframes");
    }
    if (!std::holds_alternative<std::monostate>(demand_doc.data)) {
        keys(demand_doc, {"keyframes"}, "demand program");
        for (const auto& row : io::as_array(field(demand_doc, "keyframes"), "demand keyframes")) {
            // DemandProgram.from_dict consumes these two fields; it does not
            // reject extra per-keyframe annotations in the source contract.
            out.demand_frames.push_back({io::as_double(field(row, "time_s"), "demand time"),
                                         io::as_double(field(row, "torque_nm"), "demand torque")});
        }
        if (out.demand_frames.empty()) throw std::invalid_argument("demand needs at least one keyframe");
    }
    out.validate();
    return out;
}
runtime::ResearchPrograms load_programs(const std::optional<std::filesystem::path>& rider_file,
                                       const std::optional<std::filesystem::path>& demand_file) {
    return programs_from_documents(rider_file ? configuration::detail::parse_toml_file(*rider_file) : Value{},
                                   demand_file ? configuration::detail::parse_toml_file(*demand_file) : Value{});
}
io::Value rider_program_value(const runtime::ResearchPrograms& programs) {
    if (programs.rider_frames.empty()) return Value{};
    Array frames;
    for (const auto& frame : programs.rider_frames) {
        const auto& p = frame.posture;
        Value pose{Object{{"torso_lean_rad", real(p.torso_lean_rad)}, {"pelvis_pitch_rad", real(p.pelvis_pitch_rad)},
            {"pelvis_offset_m", p.pelvis_offset_m ? reals(*p.pelvis_offset_m) : Value{}}, {"use_saddle", Value{p.use_saddle}}}};
        frames.push_back(Value{Object{{"time_s", real(frame.time_s)}, {"posture", std::move(pose)},
            {"human_torque_nm", frame.human_torque_nm ? real(*frame.human_torque_nm) : Value{}}}});
    }
    return Value{Object{{"keyframes", Value{std::move(frames)}}, {"reaction_delay_s", real(programs.reaction_delay_s)}}};
}
io::Value demand_program_value(const runtime::ResearchPrograms& programs) {
    if (programs.demand_frames.empty()) return Value{};
    Array frames;
    for (const auto& row : programs.demand_frames)
        frames.push_back(Value{Object{{"time_s", real(row[0])}, {"torque_nm", real(row[1])}}});
    return Value{Object{{"keyframes", Value{std::move(frames)}}}};
}
runtime::ResearchPrograms constant_demand(double torque_nm) {
    runtime::ResearchPrograms out;
    out.demand_frames.push_back({0., torque_nm});
    out.validate();
    return out;
}
} // namespace bike::research
