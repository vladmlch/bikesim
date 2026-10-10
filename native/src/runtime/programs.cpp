#include "programs.hpp"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <string_view>

namespace runtime {
namespace {
using namespace sample_wire;
Wire optional_wire(std::optional<double> value) { return value ? Wire(*value) : Wire(nullptr); }
std::optional<double> optional_number(const Wire &value) {
    return is_none(value) ? std::nullopt : std::optional(number(value));
}
void validate_posture(const rider::RiderPosture &p) {
    if (!std::isfinite(p.torso_lean_rad) || std::abs(p.torso_lean_rad) > .8 ||
        !std::isfinite(p.pelvis_pitch_rad) || std::abs(p.pelvis_pitch_rad) > .5)
        throw std::invalid_argument("research posture exceeds the command envelope");
    if (p.pelvis_offset_m) {
        const auto &xy = *p.pelvis_offset_m;
        if (!std::isfinite(xy[0]) || !std::isfinite(xy[1]) ||
            std::abs(xy[0]) > .25 || xy[1] < -.15 || xy[1] > .25)
            throw std::invalid_argument("research hip offset exceeds the command envelope");
    }
}
rider::RiderPosture posture_from_wire(const WireObject &value) {
    constexpr std::array<std::string_view, 4> keys{
        "torso_lean_rad", "pelvis_pitch_rad", "pelvis_offset_m", "use_saddle"};
    exact(value, keys, "research.posture");
    rider::RiderPosture p;
    p.torso_lean_rad = number(required(value, "torso_lean_rad"));
    p.pelvis_pitch_rad = number(required(value, "pelvis_pitch_rad"));
    p.use_saddle = boolean(required(value, "use_saddle"));
    const auto &offset = required(value, "pelvis_offset_m");
    if (!is_none(offset)) {
        const auto &xy = array(offset);
        if (xy.size() != 2) throw std::invalid_argument("research hip offset requires width 2");
        p.pelvis_offset_m = std::array<double, 2>{number(xy[0]), number(xy[1])};
    }
    validate_posture(p);
    return p;
}
WireObject posture_wire(const rider::RiderPosture &p) {
    return {{"torso_lean_rad", p.torso_lean_rad}, {"pelvis_pitch_rad", p.pelvis_pitch_rad},
        {"pelvis_offset_m", p.pelvis_offset_m ? wire_array(*p.pelvis_offset_m) : Wire(nullptr)},
        {"use_saddle", p.use_saddle}};
}
bool posture_equal(const rider::RiderPosture &a, const rider::RiderPosture &b) {
    return a.torso_lean_rad == b.torso_lean_rad && a.pelvis_pitch_rad == b.pelvis_pitch_rad &&
        a.pelvis_offset_m == b.pelvis_offset_m && a.use_saddle == b.use_saddle;
}
void nonnegative(double value, const char *name) {
    if (!std::isfinite(value) || value < 0.) throw std::invalid_argument(name);
}
}
WireObject research_control_wire(const RideControl &c) {
    return {{"motor_torque_nm", optional_wire(c.motor_torque_nm)},
        {"motor_limit_nm", optional_wire(c.motor_limit_nm)},
        {"human_torque_nm", optional_wire(c.human_torque_nm)},
        {"crank_target_rate_rad_s", optional_wire(c.crank_target_rate_rad_s)},
        {"posture", c.posture ? Wire(posture_wire(*c.posture)) : Wire(nullptr)},
        {"rider_enabled", c.rider_enabled}};
}
bool controls_equal(const RideControl &a, const RideControl &b) {
    return a.motor_torque_nm == b.motor_torque_nm && a.motor_limit_nm == b.motor_limit_nm &&
        a.human_torque_nm == b.human_torque_nm && a.crank_target_rate_rad_s == b.crank_target_rate_rad_s &&
        a.rider_enabled == b.rider_enabled && a.posture.has_value() == b.posture.has_value() &&
        (!a.posture || posture_equal(*a.posture, *b.posture));
}
RideControl research_control_from_wire(const WireObject &value) {
    constexpr std::array<std::string_view, 6> keys{"motor_torque_nm", "motor_limit_nm", "human_torque_nm",
        "crank_target_rate_rad_s", "posture", "rider_enabled"};
    exact(value, keys, "research.control");
    RideControl c;
    c.motor_torque_nm = optional_number(required(value, "motor_torque_nm"));
    c.motor_limit_nm = optional_number(required(value, "motor_limit_nm"));
    c.human_torque_nm = optional_number(required(value, "human_torque_nm"));
    c.crank_target_rate_rad_s = optional_number(required(value, "crank_target_rate_rad_s"));
    for (const auto item : {c.motor_torque_nm, c.motor_limit_nm, c.human_torque_nm, c.crank_target_rate_rad_s})
        if (item) nonnegative(*item, "research.control requires nonnegative finite optional values");
    c.rider_enabled = boolean(required(value, "rider_enabled"));
    const auto &p = required(value, "posture");
    if (!is_none(p)) c.posture = posture_from_wire(object(p));
    return c;
}
void ResearchPrograms::validate() const {
    nonnegative(reaction_delay_s, "rider reaction delay must be nonnegative and finite");
    double previous = -1.;
    for (const auto &frame : rider_frames) {
        nonnegative(frame.time_s, "rider keyframe time must be nonnegative and finite");
        if (frame.time_s <= previous || (previous < 0. && frame.time_s != 0.))
            throw std::invalid_argument("rider keyframes must start at zero and strictly increase");
        validate_posture(frame.posture);
        if (frame.human_torque_nm) nonnegative(*frame.human_torque_nm, "rider effort must be nonnegative and finite");
        if (frame.human_torque_nm.has_value() != rider_frames.front().human_torque_nm.has_value() ||
            frame.posture.pelvis_offset_m.has_value() != rider_frames.front().posture.pelvis_offset_m.has_value())
            throw std::invalid_argument("rider effort/offset ownership must be uniform across keyframes");
        previous = frame.time_s;
    }
    previous = -1.;
    for (const auto &frame : demand_frames) {
        nonnegative(frame[0], "demand time must be nonnegative and finite");
        nonnegative(frame[1], "demand torque must be nonnegative and finite");
        if (frame[0] <= previous || (previous < 0. && frame[0] != 0.))
            throw std::invalid_argument("demand keyframes must start at zero and strictly increase");
        previous = frame[0];
    }
}
void ResearchPrograms::validate_control(const RideControl &control) const {
    if (rider_frames.empty()) return;
    if (control.posture) throw std::invalid_argument("rider program owns posture");
    if (rider_frames.front().human_torque_nm && control.human_torque_nm)
        throw std::invalid_argument("rider program owns human effort");
}
RideControl ResearchPrograms::apply(const RideControl &control, double time_s) const {
    if (!std::isfinite(time_s)) throw std::invalid_argument("rider program time must be finite");
    validate_control(control);
    if (rider_frames.empty()) return control;
    const double t = std::max(0., time_s - reaction_delay_s);
    const auto upper = std::ranges::upper_bound(rider_frames, t, {},
        [](const RiderKeyframe &f) { return f.time_s; });
    const auto &a = *std::prev(upper);
    RideControl result = control;
    result.posture = a.posture;
    if (a.human_torque_nm) result.human_torque_nm = a.human_torque_nm;
    if (upper == rider_frames.end()) return result;
    const auto &b = *upper;
    const double u = (t - a.time_s) / (b.time_s - a.time_s);
    // The smoothstep's image is [0, 1]; at u one ulp below the upper knot
    // this association can round a hair above 1 (see RiderProgram.at).
    const double blend = std::clamp(u * u * u * (10. + u * (-15. + 6. * u)), 0., 1.);
    const auto mix = [blend](double x, double y) { return x + (y - x) * blend; };
    auto &p = *result.posture;
    p.torso_lean_rad = mix(a.posture.torso_lean_rad, b.posture.torso_lean_rad);
    p.pelvis_pitch_rad = mix(a.posture.pelvis_pitch_rad, b.posture.pelvis_pitch_rad);
    if (p.pelvis_offset_m) {
        // validate() requires uniform offset ownership across all keyframes.
        if (!a.posture.pelvis_offset_m || !b.posture.pelvis_offset_m)
            throw std::logic_error("nonuniform rider hip offset ownership");
        for (std::size_t i = 0; i < 2; ++i)
            (*p.pelvis_offset_m)[i] = mix((*a.posture.pelvis_offset_m)[i], (*b.posture.pelvis_offset_m)[i]);
    }
    if (a.human_torque_nm) {
        // validate() requires uniform effort ownership across all keyframes.
        if (!b.human_torque_nm) throw std::logic_error("nonuniform rider effort ownership");
        result.human_torque_nm = mix(*a.human_torque_nm, *b.human_torque_nm);
    }
    validate_posture(p);
    return result;
}
std::optional<double> ResearchPrograms::demand_at(double time_s) const {
    if (!std::isfinite(time_s)) throw std::invalid_argument("demand time must be finite");
    if (demand_frames.empty()) return std::nullopt;
    const double t = std::max(0., time_s);
    const auto upper = std::ranges::upper_bound(demand_frames, t, {},
        [](const std::array<double, 2> &f) { return f[0]; });
    const auto &a = *std::prev(upper);
    if (upper == demand_frames.end()) return a[1];
    const auto &b = *upper;
    const double u = (t - a[0]) / (b[0] - a[0]);
    // demand.py associates the difference before u*u*u; do NOT reuse the
    // separately rounded rider blend here.
    return a[1] + (b[1] - a[1]) * u * u * u * (10. + u * (-15. + 6. * u));
}
ResearchPrograms ResearchPrograms::from_wire(const Wire &rider_program, const Wire &demand_program) {
    ResearchPrograms result;
    if (!is_none(rider_program)) {
        const auto &p = object(rider_program);
        constexpr std::array<std::string_view, 2> keys{"keyframes", "reaction_delay_s"};
        exact(p, keys, "rider_program");
        result.reaction_delay_s = number(required(p, "reaction_delay_s"));
        for (const auto &item : array(required(p, "keyframes"))) {
            const auto &f = object(item);
            constexpr std::array<std::string_view, 3> fields{"time_s", "posture", "human_torque_nm"};
            exact(f, fields, "rider_program.keyframes");
            result.rider_frames.push_back(RiderKeyframe{.time_s = number(required(f, "time_s")),
                .posture = posture_from_wire(object(required(f, "posture"))),
                .human_torque_nm = optional_number(required(f, "human_torque_nm"))});
        }
        if (result.rider_frames.empty()) throw std::invalid_argument("rider program requires keyframes");
    }
    if (!is_none(demand_program)) {
        const auto &p = object(demand_program);
        constexpr std::array<std::string_view, 1> keys{"keyframes"};
        exact(p, keys, "demand_program");
        for (const auto &item : array(required(p, "keyframes"))) {
            const auto &f = object(item);
            constexpr std::array<std::string_view, 2> fields{"time_s", "torque_nm"};
            exact(f, fields, "demand_program.keyframes");
            result.demand_frames.push_back({number(required(f, "time_s")), number(required(f, "torque_nm"))});
        }
        if (result.demand_frames.empty()) throw std::invalid_argument("demand program requires keyframes");
    }
    result.validate();
    return result;
}
} // namespace runtime
