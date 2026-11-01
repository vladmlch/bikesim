// Completion 3.33a. Source: tests/reference/test_native_research_programs.py.
// Rider/demand program interpolation at the knot boundaries — frozen to
// per-platform goldens — plus the no-program passthrough and the
// mixed-ownership rejection contracts.
//
// Substitutions: asdict(RideControl)/to_dict() dicts are built as Wire
// literals and decoded through ResearchPrograms::from_wire +
// research_control_from_wire — the same decode family the binding's
// research_program_at runs (parse_intent_control is nanobind-only; the wire
// decoder below is its field-set twin).
#include <cstddef>

#include "research_reference_support.hpp"
#include "support/golden.hpp"

namespace {
using namespace research_reference;
namespace sw = runtime::sample_wire;

runtime::Wire none_wire() { return runtime::Wire{nullptr}; }
runtime::Wire opt_wire(std::optional<double> value) {
    return value ? runtime::Wire{*value} : runtime::Wire{nullptr};
}
runtime::Wire posture_dict(std::optional<std::array<double, 2>> offset,
                           double torso_lean = 0., double pelvis_pitch = 0.,
                           bool use_saddle = true) {
    return runtime::Wire{runtime::WireObject{
        {"torso_lean_rad", runtime::Wire{torso_lean}},
        {"pelvis_pitch_rad", runtime::Wire{pelvis_pitch}},
        {"pelvis_offset_m",
         offset ? runtime::Wire{runtime::WireArray{
                      runtime::Wire{(*offset)[0]}, runtime::Wire{(*offset)[1]}}}
                : runtime::Wire{nullptr}},
        {"use_saddle", runtime::Wire{use_saddle}}}};
}
runtime::Wire keyframe_dict(double time_s, runtime::Wire posture,
                            std::optional<double> human_torque) {
    return runtime::Wire{runtime::WireObject{
        {"time_s", runtime::Wire{time_s}},
        {"posture", std::move(posture)},
        {"human_torque_nm", opt_wire(human_torque)}}};
}
runtime::Wire rider_program_dict(std::optional<std::array<double, 2>> offsets,
                                 std::optional<double> effort) {
    return runtime::Wire{runtime::WireObject{
        {"keyframes", runtime::Wire{runtime::WireArray{
            keyframe_dict(0., posture_dict(offsets), effort),
            keyframe_dict(.031, posture_dict(offsets, .1, -.02, false),
                          effort ? std::optional(*effort + 3.) : std::nullopt),
            keyframe_dict(.067, posture_dict(offsets, -.07, .03, true), effort)}}},
        {"reaction_delay_s", runtime::Wire{.004}}}};
}
runtime::Wire demand_program_dict() {
    return runtime::Wire{runtime::WireObject{
        {"keyframes", runtime::Wire{runtime::WireArray{
            runtime::Wire{runtime::WireObject{
                {"time_s", runtime::Wire{0.}},
                {"torque_nm", runtime::Wire{1.125}}}},
            runtime::Wire{runtime::WireObject{
                {"time_s", runtime::Wire{.017}},
                {"torque_nm", runtime::Wire{12.75}}}},
            runtime::Wire{runtime::WireObject{
                {"time_s", runtime::Wire{.049}},
                {"torque_nm", runtime::Wire{2.0625}}}}}}}}};
}
runtime::Wire control_dict(std::optional<double> motor, std::optional<double> limit,
                           std::optional<double> human, std::optional<double> crank,
                           runtime::Wire posture, bool rider_enabled) {
    return runtime::Wire{runtime::WireObject{
        {"motor_torque_nm", opt_wire(motor)},
        {"motor_limit_nm", opt_wire(limit)},
        {"human_torque_nm", opt_wire(human)},
        {"crank_target_rate_rad_s", opt_wire(crank)},
        {"posture", std::move(posture)},
        {"rider_enabled", runtime::Wire{rider_enabled}}}};
}
runtime::Wire malformed_rider_dict() {
    // keyframes[1].human_torque_nm = None against a uniform-effort program.
    return runtime::Wire{runtime::WireObject{
        {"keyframes", runtime::Wire{runtime::WireArray{
            keyframe_dict(0., posture_dict(std::nullopt), 0.),
            keyframe_dict(1., posture_dict(std::nullopt), std::nullopt)}}},
        {"reaction_delay_s", runtime::Wire{0.}}}};
}

// The flattened 18-wide golden row: 16 control values + 2 demand values,
// presence flags first — identical to the Python capture site's layout.
void flatten_control(const runtime::RideControl &control, std::vector<double> &out) {
    const auto opt = [&out](const std::optional<double> &value) {
        out.push_back(value ? 1. : 0.);
        out.push_back(value.value_or(0.));
    };
    opt(control.motor_torque_nm);
    opt(control.motor_limit_nm);
    opt(control.human_torque_nm);
    opt(control.crank_target_rate_rad_s);
    out.push_back(control.posture ? 1. : 0.);
    out.push_back(control.posture ? control.posture->torso_lean_rad : 0.);
    out.push_back(control.posture ? control.posture->pelvis_pitch_rad : 0.);
    out.push_back(control.posture && control.posture->pelvis_offset_m ? 1. : 0.);
    out.push_back(control.posture && control.posture->pelvis_offset_m
                      ? (*control.posture->pelvis_offset_m)[0]
                      : 0.);
    out.push_back(control.posture && control.posture->pelvis_offset_m
                      ? (*control.posture->pelvis_offset_m)[1]
                      : 0.);
    out.push_back(control.posture && control.posture->use_saddle ? 1. : 0.);
    out.push_back(control.rider_enabled ? 1. : 0.);
}
void flatten_result(const runtime::RideControl &control,
                    std::optional<double> demand, std::vector<double> &out) {
    flatten_control(control, out);
    out.push_back(demand ? 1. : 0.);
    out.push_back(demand.value_or(0.));
}

// tests/reference/test_native_research_programs.py::
//   test_program_knots_optional_fields_and_association
void test_program_knots_optional_fields_and_association(
    const char *node_suffix, std::optional<std::array<double, 2>> offsets,
    std::optional<double> effort) {
    const auto programs = runtime::ResearchPrograms::from_wire(
        rider_program_dict(offsets, effort), demand_program_dict());
    const auto command = runtime::research_control_from_wire(sw::object(
        control_dict(std::nullopt, 0., effort ? std::nullopt : std::optional(4.),
                     0., none_wire(), false)));
    const double below =
        std::nextafter(.035, -std::numeric_limits<double>::infinity());
    const double above =
        std::nextafter(.035, std::numeric_limits<double>::infinity());
    const std::array<double, 10> times{
        -.01, 0., .004, .0123, .021, .035, .071, 1., below, above};
    std::vector<double> rows;
    rows.reserve(static_cast<std::size_t>(10 * 18));
    for (const double time_s : times) {
        const auto result = programs.apply(command, time_s);
        flatten_result(result, programs.demand_at(time_s), rows);
        bt::require(!result.motor_torque_nm.has_value() &&
                        result.motor_limit_nm == 0.,
                    "motor torque stays absent and the limit stays zero");
    }
    bt::oracle::Node node(
        "tests/reference/test_native_research_programs.py::"
        "test_program_knots_optional_fields_and_association" +
        std::string(node_suffix));
    const std::vector<std::size_t> shape{10, 18};
    node.f64(rows, shape, "program_at");
    node.done();
}

// tests/reference/test_native_research_programs.py::
//   test_no_program_preserves_none_and_explicit_zero
void test_no_program_preserves_none_and_explicit_zero() {
    const runtime::ResearchPrograms programs{};
    // RideControl() — every field absent, rider_enabled default true.
    const auto empty = programs.apply(runtime::RideControl{}, 0.);
    bt::require(!empty.motor_torque_nm && !empty.motor_limit_nm &&
                    !empty.human_torque_nm && !empty.crank_target_rate_rad_s &&
                    !empty.posture && empty.rider_enabled,
                "an empty program preserves an empty command");
    // RideControl(0., 0., 0., 0., RiderPosture(), False) — every field
    // explicit, including the zero values.
    runtime::RideControl const full{.motor_torque_nm = 0.,
                                    .motor_limit_nm = 0.,
                                    .human_torque_nm = 0.,
                                    .crank_target_rate_rad_s = 0.,
                                    .posture = rider::RiderPosture{},
                                    .rider_enabled = false};
    const auto kept = programs.apply(full, 0.);
    bt::require(kept.motor_torque_nm == 0. && kept.motor_limit_nm == 0. &&
                    kept.human_torque_nm == 0. && kept.crank_target_rate_rad_s == 0. &&
                    kept.posture.has_value() && !kept.rider_enabled,
                "an empty program preserves explicit zeroes and the posture");
    bt::require(!programs.demand_at(0.).has_value(),
                "an empty program reports no demand");
}

// tests/reference/test_native_research_programs.py::
//   test_reject_mixed_effort_and_policy_posture
void test_reject_mixed_effort_and_policy_posture() {
    // RiderProgram((0., effort 0.), (1., effort 2.)) — effort-owning program.
    const auto programs = runtime::ResearchPrograms::from_wire(
        runtime::Wire{runtime::WireObject{
            {"keyframes", runtime::Wire{runtime::WireArray{
                keyframe_dict(0., posture_dict(std::nullopt), 0.),
                keyframe_dict(1., posture_dict(std::nullopt), 2.)}}},
            {"reaction_delay_s", runtime::Wire{0.}}}},
        none_wire());
    // Control with human effort collides with program ownership.
    bt::invalid([&] {
        runtime::RideControl command;
        command.human_torque_nm = 0.;
        (void)programs.apply(command, .5); });
    // Control with a posture collides with program ownership.
    bt::invalid([&] {
        runtime::RideControl command;
        command.posture = rider::RiderPosture{};
        (void)programs.apply(command, .5); });
    // Mixed effort ownership across keyframes is malformed at decode.
    invalid_with([&] {
        (void)runtime::ResearchPrograms::from_wire(malformed_rider_dict(),
                                                 none_wire()); },
                 "uniform");
}

} // namespace

int main() {
    return bt::run([] {
        test_program_knots_optional_fields_and_association(
            "[None-None]", std::nullopt, std::nullopt);
        test_program_knots_optional_fields_and_association(
            "[None-offsets1]", std::array<double, 2>{0., 0.}, std::nullopt);
        test_program_knots_optional_fields_and_association(
            "[0.0-None]", std::nullopt, 0.);
        test_program_knots_optional_fields_and_association(
            "[0.0-offsets1]", std::array<double, 2>{0., 0.}, 0.);
        test_program_knots_optional_fields_and_association(
            "[12.5-None]", std::nullopt, 12.5);
        test_program_knots_optional_fields_and_association(
            "[12.5-offsets1]", std::array<double, 2>{0., 0.}, 12.5);
        test_no_program_preserves_none_and_explicit_zero();
        test_reject_mixed_effort_and_policy_posture();
    });
}
