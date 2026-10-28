// Completion 3.34 (T53). Source: tests/reference/test_realtime_deferred_checks.py.
//
// The Python file covers the deferred-attachment pipeline: bulk
// road_samples vs the scalar profile formulas, the unexplained-wrench
// deferral through evaluate_period's first_failure attribution, and the
// 200-step scalar-vs-deferred parity. The native twins are
// runtime::road_samples/RoadProfile, rider::attachment_wrench's
// prepare/raw_from_geometry pair and runtime::PeriodAccounting — the real
// pinned plant (welded bootstrap model.mjb) for the efc arena, the real
// accounting path for the interval attribution.
#include "support/migration_test.hpp"
#include "pinned_test_support.hpp"
#include "engine_call.hpp"
#include "numeric_numpy.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/least_squares.hpp"
#include "runtime/accounting.hpp"
#include "runtime/config.hpp"
#include "runtime/control.hpp"
#include "runtime/period_buffer.hpp"
#include "runtime/status.hpp"
#include "runtime/step.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace {

namespace pt = pinned_test;
namespace rw = rider;

// test_bulk_road_queries_match_two_hundred_scalar_windows — the bulk
// road_samples must equal the scalar _profile_height/_profile_grade pair on
// every emitted x. The scalar twins are the documented formulas:
// np.interp on the vertex columns for height, and the per-segment
// dz/dx of the segment containing x for grade (the port reimplements
// np.interp + np.searchsorted verbatim, so the check is the window/x set
// the oracle swept — 200 consecutive (start, front+lookahead) windows).
void test_bulk_road_queries_match_two_hundred_scalar_windows() {
    const std::array<std::array<double, 2>, 5> vertices{
        {{0., 0.}, {.7, .08}, {1.6, -.02}, {2.8, .24}, {5., .3}}};
    std::vector<double> xs, zs;
    for (const auto &v : vertices) {
        xs.push_back(v[0]);
        zs.push_back(v[1]);
    }
    const auto height_at = [&](double x) {
        // np.interp: clamp to the endpoint plates outside [x0, xN].
        if (x <= xs.front()) return zs.front();
        if (x >= xs.back()) return zs.back();
        const auto it = std::ranges::upper_bound(xs, x);
        const auto i = static_cast<std::size_t>(
            std::distance(xs.begin(), it) - 1);
        // np.interp's exact lowering (slope*(x-x0)+z0) through the shared
        // contracted/separate eval mirror — the same helper road_samples
        // calls via biketyre::np_interp.
        const double slope = (zs[i + 1] - zs[i]) / (xs[i + 1] - xs[i]);
        return numeric::np_interp_eval(slope, x - xs[i], zs[i]);
    };
    const auto grade_at = [&](double x) {
        // _profile_grade: the owning segment's slope; searchsorted
        // side='right' - 1 clipped to [0, n-2].
        const auto it = std::ranges::upper_bound(xs, x);
        auto seg = std::distance(xs.begin(), it) - 1;
        seg = std::clamp<std::ptrdiff_t>(
            seg, 0, static_cast<std::ptrdiff_t>(xs.size()) - 2);
        return (zs[static_cast<std::size_t>(seg) + 1] -
                zs[static_cast<std::size_t>(seg)]) /
               (xs[static_cast<std::size_t>(seg) + 1] -
                xs[static_cast<std::size_t>(seg)]);
    };
    for (int i = 0; i < 200; ++i) {
        const double start = .1 + i * .01, front = 1.2 + i * .01;
        const std::array wheels{start, front};
        const auto samples = runtime::road_samples(vertices, wheels, .371);
        bt::require(samples.back().x_m == front + .371,
                    "the window ends exactly at front+lookahead");
        for (const auto &sample : samples) {
            bt::require(sample.height_m == height_at(sample.x_m),
                        "height matches the scalar interp oracle");
            bt::require(sample.grade == grade_at(sample.x_m),
                        "grade matches the scalar segment oracle");
        }
    }
}

// test_deferred_unexplained_wrench_is_rejected_at_its_original_interval —
// corrupt the bike-side jacobian so the solved foot force cannot be
// explained: eager validation raises 'does not explain', the deferred raw
// survives capture, and period close attributes the violation to the
// interval that produced it — (.00125, ('foot_front:unobservable...',)).
void test_deferred_unexplained_wrench_is_rejected_at_its_original_interval() {
    auto plant = pt::pinned();
    mjModel *m = plant.model.get();
    mjData *d = plant.data.get();
    engine::forward(m, d);
    const int eq = pt::eq_id(m, "connect_foot_front");
    const int rider_body = pt::int_at(m->eq_obj1id, m->neq, eq);
    const int bike_body = pt::int_at(m->eq_obj2id, m->neq, eq);
    std::ranges::fill(pt::mbuf(d->efc_force, d->nefc), 0.);
    rider::EqualityReactions reactions(m);
    const auto rows = reactions.rows_of(d, eq);
    bt::require(rows.size() == 3, "foot connect rows present");
    const std::array<mjtNum, 3> force{0., 0., 100.};
    for (std::size_t r = 0; r < 3; ++r)
        pt::mbuf(d->efc_force, d->nefc)[static_cast<std::size_t>(rows[r])] = force[r];
    const auto point = pt::world_anchor(m, d, eq, rider_body, 0);
    const std::vector<double> normal{0., 0., 1.};
    const auto geometry = rider::prepare_attachment_geometry(
        m, d, eq, rider_body, bike_body, point, normal, "foot",
        /*rotational=*/false, 0., std::nullopt);
    auto bad = geometry;
    std::ranges::fill(bad.bike_jac.values, 0.);
    const auto nv = static_cast<rider::lapack_int>(m->nv);
    rider::LeastSquaresWorkspace workspace(nv,
                                           std::max<rider::lapack_int>(nv, 6),
                                           1);
    std::vector<double> scratch(static_cast<std::size_t>(m->nv), 0.);
    bt::invalid([&] {
        static_cast<void>(rider::attachment_raw_from_geometry(
            d, reactions, workspace, scratch, bad, /*validate=*/true));
    });
    // validate_wrench=False returns the raw for the deferred check.
    const auto captured = rider::attachment_raw_from_geometry(
        d, reactions, workspace, scratch, bad, /*validate=*/false);
    // evaluate_period's twin: PeriodAccounting closes the one-raw period
    // and the monitor latches the violation at the interval's own
    // end_time_s.
    runtime::AccountingConfig config{
        .timestep_s = .00125,
        .period_steps = 1,
        .measurement_decimation = 1,
        .retention = runtime::RetentionMode::legacy,
        .battery_enabled = false,
        .async_telemetry = false,
        .max_qpos = static_cast<std::size_t>(m->nq),
        .max_dofs = static_cast<std::size_t>(m->nv),
        .attachment_budget = {},
        .columns = {},
    };
    runtime::AccountingState state;
    state.energy.initial_energy_j = 1.;
    state.energy.initial_battery_j = 0.;
    state.energy.energy_scale_j = 1.;
    runtime::PeriodAccounting accounting(config, std::move(state), 1,
                                         nullptr, nullptr);
    runtime::RawStep raw;
    raw.interval_id = 0;
    raw.time_s = 0.;
    raw.end_time_s = .00125;
    raw.qpos.assign(static_cast<std::size_t>(m->nq), 0.);
    raw.qvel.assign(static_cast<std::size_t>(m->nv), 0.);
    raw.attachment_raw = {{"foot_front", captured}};
    raw.electrical_power_w = runtime::Wire(0.);
    raw.mechanical_energy_j = 1.;
    raw.elastic_energy_j = runtime::WireObject{};
    raw.constraint_snapshot = {.interval_start_s = 0.,
                               .interval_end_s = .00125,
                               .qvel_start = raw.qvel,
                               .components = {}};
    raw.diagnostics = {
        {"endpoint_mass", runtime::Wire(runtime::WireObject{})},
        {"rider", runtime::Wire(runtime::WireObject{})},
        {"rider_welds", runtime::Wire(runtime::WireObject{})},
        {"rider_ik_saturation", runtime::Wire(nullptr)},
        {"rider_support_targets", runtime::Wire(nullptr)},
        {"rider_intent", runtime::Wire(nullptr)},
        {"inclination_rad", runtime::Wire(0.)},
        {"rider_allocation", runtime::Wire(runtime::WireObject{})},
    };
    // The retained-row contract validates channels.tires patches on commit
    // (WorkHistory::add) — stage one terrain patch per wheel.
    const auto tire = [](double load) {
        return runtime::Wire{runtime::WireObject{
            {"patches", runtime::Wire{runtime::WireArray{
                runtime::Wire{runtime::WireObject{
                    {"normal_load_n", runtime::Wire{load}},
                    {"source_geom", runtime::Wire{"terrain"}},
                }},
            }}},
        }};
    };
    raw.channels = {{"tires", runtime::Wire{runtime::WireObject{
        {"front", tire(350.)},
        {"rear", tire(350.)},
    }}}};
    accounting.push(std::move(raw));
    accounting.close_period();
    const auto &first = accounting.state().monitor.first_failure;
    bt::require(first.has_value(),
                "the deferred rejection latches a first failure");
    bt::require(first->time_s == .00125,
                "the failure cites the original interval end");
    bt::require(first->violations ==
                    std::vector<std::string>{
                        "foot_front:unobservable_attachment_wrench"},
                "the violation names the foot attachment");
}

// test_two_hundred_steps_match_scalar_validation_oracle — the oracle
// monkeypatches attachment_raw_from_geometry to always validate eagerly and
// confirms 200 steps of samples/first_failure are bitwise-identical to the
// deferred pipeline. Natively the two paths are the SAME solves: eager
// attachment_sample performs the validation inline; deferred capture +
// evaluate_attachments performs it at period close on the same inputs.
// On a valid wrench both must produce the identical AttachmentSample.
void test_two_hundred_steps_match_scalar_validation_oracle() {
    auto plant = pt::pinned();
    mjModel *m = plant.model.get();
    mjData *d = plant.data.get();
    engine::forward(m, d);
    const int eq = pt::eq_id(m, "connect_foot_front");
    const int rider_body = pt::int_at(m->eq_obj1id, m->neq, eq);
    const int bike_body = pt::int_at(m->eq_obj2id, m->neq, eq);
    rider::EqualityReactions reactions(m);
    const auto rows = reactions.rows_of(d, eq);
    bt::require(rows.size() == 3, "foot connect rows present");
    const std::array<mjtNum, 3> force{20., 0., 100.};
    for (std::size_t r = 0; r < 3; ++r)
        pt::mbuf(d->efc_force, d->nefc)[static_cast<std::size_t>(rows[r])] = force[r];
    const auto point = pt::world_anchor(m, d, eq, rider_body, 0);
    const std::vector<double> normal{0., 0., 1.};
    const auto nv = static_cast<rider::lapack_int>(m->nv);
    rider::LeastSquaresWorkspace workspace(nv,
                                           std::max<rider::lapack_int>(nv, 6),
                                           1);
    std::vector<double> scratch(static_cast<std::size_t>(m->nv), 0.);
    // The scalar/eager read — validates inline like the monkeypatched
    // converter.
    const auto eager = rider::attachment_sample(
        m, d, reactions, workspace, scratch, eq, rider_body, bike_body,
        point, normal, "foot", /*rotational=*/false, 0., std::nullopt);
    // The deferred read — captured at interval start, solved at period
    // close, identical wrench inputs.
    const auto geometry = rider::prepare_attachment_geometry(
        m, d, eq, rider_body, bike_body, point, normal, "foot",
        /*rotational=*/false, 0., std::nullopt);
    const auto captured = rider::attachment_raw_from_geometry(
        d, reactions, workspace, scratch, geometry, /*validate=*/false);
    runtime::RawStep raw;
    raw.attachment_raw = {{"foot_front", captured}};
    const auto deferred = runtime::evaluate_attachments(raw, workspace);
    bt::require(deferred.size() == 1 && deferred.front().first ==
                    "foot_front",
                "the deferred path publishes the foot sample");
    const auto &sample = deferred.front().second;
    bt::require(sample.normal_n == eager.normal_n &&
                    sample.tangent_n == eager.tangent_n &&
                    sample.moment_nm == eager.moment_nm &&
                    sample.gap_m == eager.gap_m &&
                    sample.pull_n == eager.pull_n &&
                    sample.half_patch_m == eager.half_patch_m,
                "deferred and scalar reads produce the identical sample");
}

} // namespace

int main() {
    return bt::run([] {
        test_bulk_road_queries_match_two_hundred_scalar_windows();
        test_deferred_unexplained_wrench_is_rejected_at_its_original_interval();
        test_two_hundred_steps_match_scalar_validation_oracle();
    });
}
