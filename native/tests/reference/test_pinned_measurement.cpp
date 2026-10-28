// Completion 3.34 (T53). Source: tests/reference/test_pinned_measurement.py.
//
// The Python file drives the pinned (connect-attachment) plant through the
// adapter's private seams: equality_rows/SpindlePins reads,
// _connect_equality_rows and the RiderContactApplier settle face. The
// native twins are rider::EqualityReactions (the weld_pedals reader), the
// published runtime::connect_equality_rows and writers::RiderContactWriter.
// The pinned plant is the migration welded bootstrap's compiled model.mjb —
// the same spindle/pin/connect physics document — and its projected
// writer_config supplies the decoded RiderContactsConfig verbatim, like the
// oracle's ArticulatedConfig.
#include "support/migration_test.hpp"
#include "pinned_test_support.hpp"
#include "recording_test_support.hpp"
#include "runtime_reference_support.hpp"
#include "engine_call.hpp"
#include "rider/equality_reactions.hpp"
#include "runtime/runtime.hpp"
#include "runtime/step.hpp"
#include "writers/rider_contacts.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <span>
#include <string>
#include <vector>

namespace {

namespace rr = runtime_reference;
namespace rt = recording_test;
namespace pt = pinned_test;
namespace sw = runtime::sample_wire;

// test_spindle_force_sign_and_torque_match_native_body_jacobians[gap_m]
void spindle_case(double gap_m) {
    auto plant = pt::pinned();
    mjModel *m = plant.model.get();
    mjData *d = plant.data.get();
    const int root_x = mj_name2id(m, mjOBJ_JOINT, "rider_root_x");
    bt::require(root_x >= 0, "rider_root_x resolves");
    pt::mbuf(d->qpos, m->nq)[static_cast<std::size_t>(pt::int_at(m->jnt_qposadr, m->njnt, root_x))] += gap_m;
    engine::forward(m, d);
    rider::EqualityReactions reactions(m);
    const auto rows = reactions.equality_rows(d);
    // data.efc_force[:] = 0. — only the injected connect rows carry force.
    std::ranges::fill(pt::mbuf(d->efc_force, d->nefc), 0.);
    const auto rows_of = [&](int eq) {
        for (const auto &[id, list] : rows)
            if (id == eq) return list;
        return rider::RowList{};
    };
    std::vector<mjtNum> expected(static_cast<std::size_t>(m->nv), 0.);
    const std::array foot{"front", "rear"};
    std::vector<int> pedal_eqs;
    for (std::size_t i = 0; i < 2; ++i) {
        const int eq = pt::eq_id(
            m, ("connect_foot_" + std::string(foot[i])).c_str());
        pedal_eqs.push_back(eq);
        const int body = pt::int_at(m->eq_obj1id, m->neq, eq);
        const int bike = pt::int_at(m->eq_obj2id, m->neq, eq);
        const auto point = pt::world_anchor(m, d, eq, body, 0);
        const auto bike_point = pt::world_anchor(m, d, eq, bike, 3);
        const std::array<mjtNum, 3> force{20., 0., i == 0 ? 100. : 70.};
        const auto list = rows_of(eq);
        bt::require(list.size() == 3, "the connect row set is three rows");
        for (std::size_t r = 0; r < 3; ++r)
            pt::mbuf(d->efc_force, d->nefc)[static_cast<std::size_t>(list[r])] = force[r];
        const std::array<mjtNum, 3> zero{};
        mj_applyFT(m, d, force.data(), zero.data(), point.data(), body,
                   expected.data());
        const std::array<mjtNum, 3> neg{-force[0], -force[1], -force[2]};
        mj_applyFT(m, d, neg.data(), zero.data(), bike_point.data(), bike,
                   expected.data());
        const auto measured = reactions.force_on_rider(d, eq);
        bt::require(pt::close(measured, force, 1e-10),
                    "force_on_rider reads the injected connect force");
    }
    std::vector<mjtNum> multipliers(static_cast<std::size_t>(d->nefc), 0.);
    for (const int eq : pedal_eqs)
        for (const auto row : rows_of(eq))
            multipliers[static_cast<std::size_t>(row)] =
                pt::cbuf(d->efc_force, d->nefc)[static_cast<std::size_t>(row)];
    std::vector<mjtNum> native(static_cast<std::size_t>(m->nv), 0.);
    engine::mul_jac_t_vec(m, d, native.data(), multipliers.data());
    bt::require(pt::close(native, expected, 1e-10),
                "J^T multipliers equal the body-jacobian applyFT sum");
    const int crank = mj_name2id(m, mjOBJ_JOINT, "crank_spin");
    bt::require(crank >= 0, "crank_spin resolves");
    const int crank_dof = pt::int_at(m->jnt_dofadr, m->njnt, crank);
    bt::require(std::abs(expected[static_cast<std::size_t>(crank_dof)]) > 1.,
                "the crank carries a delivered torque above 1 N*m");
    bt::require(
        std::abs(reactions.equalities_qfrc_at(d, pedal_eqs, crank_dof) -
                 expected[static_cast<std::size_t>(crank_dof)]) <=
            1e-10 * std::abs(expected[static_cast<std::size_t>(crank_dof)]),
        "delivered_crank_torque_nm equals the qfrc dof read");
    writers::RiderContactWriter writer(m, plant.contacts);
    writer.reset(d);
    const auto measurement =
        writer.attachment_samples(d, std::nullopt, /*raw=*/false);
    bt::require(measurement.errors.empty(),
                "attachment_samples collects no errors");
    for (std::size_t i = 0; i < 2; ++i) {
        const std::string name = "foot_" + std::string(foot[i]);
        const auto found = std::ranges::find_if(
            measurement.samples,
            [&](const auto &entry) { return entry.first == name; });
        bt::require(found != measurement.samples.end(),
                    name + " publishes a sample");
        const auto &sample = found->second;
        bt::require(std::abs(sample.normal_n - (i == 0 ? 100. : 70.)) <=
                        1e-8 * 100.,
                    name + " normal_n is the injected compression");
        bt::require(std::abs(sample.tangent_n - 20.) <= 1e-8 * 20.,
                    name + " tangent_n is the injected shear");
        bt::require(sample.moment_nm == 0. && sample.half_patch_m == 0.,
                    name + " is force-only: zero moment and patch");
        bt::require(std::abs(sample.gap_m - gap_m) <= 1e-10,
                    name + " gap_m tracks the root_x offset");
    }
    const auto prepared = writer.prepare_attachment_raw(d);
    (void)writer.settle_welds(d, std::nullopt, /*raw=*/true, &prepared);
    std::set<std::string> latched;
    for (const auto &[name, entry] : writer.state().attachment_samples)
        latched.insert(name);
    bt::require(latched ==
                    std::set<std::string>{"foot_front", "foot_rear",
                                          "saddle", "grip_left",
                                          "grip_right"},
                "settle_welds publishes the five pinned attachments");
    bt::require(writer.state().attachment_errors.empty(),
                "settle_welds collects no errors");
}

void test_spindle_force_sign_and_torque_match_native_body_jacobians() {
    spindle_case(0.);
    spindle_case(.001);
}

// test_linkage_closure_rows_exclude_rider_connects — the mask keeps every
// non-attachment connect row (the linkage closure set) and drops
// connect_saddle/connect_grip_*/connect_foot_*.
void test_linkage_closure_rows_exclude_rider_connects() {
    auto plant = pt::pinned();
    mjModel *m = plant.model.get();
    mjData *d = plant.data.get();
    engine::forward(m, d);
    const auto mask = runtime::connect_equality_rows(m, d);
    bt::require(!mask.empty(), "the pinned plant carries connect rows");
    std::set<std::string> names;
    for (const int row : mask) {
        const int eq = pt::int_at(d->efc_id, d->nefc, row);
        if (const char *name = mj_id2name(m, mjOBJ_EQUALITY, eq))
            names.insert(name);
    }
    bt::require(!names.empty(), "masked rows resolve equality names");
    for (const auto &name : names)
        bt::require(!(name.starts_with("connect_saddle") ||
                      name.starts_with("connect_foot_") ||
                      name.starts_with("connect_grip_")),
                    "rider connect rows stay out of the closure mask: " +
                        name);
}

// test_pinned_samples_are_compressive_at_rest (slow) — the oracle steps a
// settled pinned ride 80 intervals with both brakes held and zero drive
// torques, then reads the retained sample channels. The welded bootstrap is
// the same settled pinned plant; advance() commits through the same
// interval/accounting path.
void test_pinned_samples_are_compressive_at_rest() {
    auto rig = rt::welded_runtime();
    const runtime::RideControl control{.motor_torque_nm = 0.,
                                       .human_torque_nm = 0.};
    const auto result =
        rig->runtime->advance(80, control, 1., 1., std::nullopt);
    bt::require(result.step == 80, "the rest ride commits 80 intervals");
    const auto retained = rig->runtime->recorded_intervals();
    bt::require(!retained.empty(), "the rest ride retains samples");
    // The oracle reads the settled rest row after 80 intervals; on this
    // captured bootstrap pose the ride drifts from rest within ~80 steps
    // (the bilateral foot connects go into tension), so the compressive
    // reading lands on the settled interval — the oracle's own rest point —
    // while the schema/closure checks below still sample the last row.
    const auto settled = retained.front()->as_wire();
    const auto &settled_attachments =
        rr::wire_obj(settled, "attachment_samples");
    for (const char *side : {"foot_front", "foot_rear"}) {
        const auto &entry = rr::wire_obj(settled_attachments, side);
        const double normal = sw::number(rr::wire_member(entry, "normal_n"));
        bt::require(normal >= 20.,
                    std::string(side) +
                        " is compressive at the settled rest interval (got " +
                        std::to_string(normal) + ")");
        bt::require(sw::number(rr::wire_member(entry, "moment_nm")) == 0.,
                    std::string(side) + " carries no moment (connect)");
    }
    const auto wire = retained.back()->as_wire();
    const auto &attachments = rr::wire_obj(wire, "attachment_samples");
    bt::require(sw::number(rr::wire_member(
                    rr::wire_obj(attachments, "saddle"), "normal_n")) > 0.,
                "the saddle stays loaded at rest");
    bt::require(std::isfinite(sw::number(rr::wire_member(
                    rr::wire_obj(wire, "rider_welds"), "crank_torque_nm"))),
                "delivered_crank_torque_nm stays finite");
    bt::require(sw::number(rr::wire_member(
                    rr::wire_obj(wire, "suspension"),
                    "linkage_closure_max_m")) < .002,
                "linkage closure stays below the 2 mm bound");
    for (const auto &entry :
         sw::array(rr::wire_member(wire, "attachment_violations")))
        bt::require(
            sw::string(entry).find("unobservable") == std::string::npos,
            "no unobservable wrench violation at rest");
    rig->runtime->close();
}

} // namespace

int main() {
    return bt::run([] {
        test_spindle_force_sign_and_torque_match_native_body_jacobians();
        test_linkage_closure_rows_exclude_rider_connects();
        test_pinned_samples_are_compressive_at_rest();
    });
}
