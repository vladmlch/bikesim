// Completion 3.14b (T52). Source: tests/reference/test_attachment_wrench.py.
// The Python-oracle attachment-wrench algebra — recover_wrench solves,
// weld/connect reactions, equality-row selection under interleaved
// contacts, and the prepared-geometry/eager-raw equivalence — replayed
// against the platform goldens. The two 'foot cop budget' source records
// exercise RiderContactApplier's Python-internal _attachment_samples
// plumbing and are dispositioned `drop` in the port map: the native
// half-patch channel is pinned through the 3.14b attachment_sample
// oracle nodes instead.

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <functional>
#include <ranges>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "diag.hpp"
#include "engine_call.hpp"
#include "support/golden.hpp"
#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/least_squares.hpp"

namespace {
namespace rw = rider;
using rider::DenseMatrix;

constexpr std::string_view kFile =
    "tests/reference/test_attachment_wrench.py::";

void scalar(bt::oracle::Node &golden, double actual, std::string_view label) {
    const std::array<double, 1> value{actual};
    golden.f64_close(value, std::span<const std::size_t>{}, label);
}

void error_bytes(bt::oracle::Node &golden, const std::exception &error) {
    const std::string_view text = error.what();
    const std::span<const std::byte> bytes =
        std::as_bytes(std::span{text});
    const std::array<std::size_t, 1> shape{text.size()};
    golden.bytes(bytes, "|u1", shape, "error");
}

// xfrc_applied row view for one body: six mjtNum slots starting at
// 6*body_id.
std::span<mjtNum> xfrc_row(const mjModel *m, mjData *d, int body) {
    const std::span<mjtNum> all =
        std::views::counted(d->xfrc_applied, 6 * m->nbody);
    return all.subspan(static_cast<std::size_t>(6 * body), 6);
}

double dot(std::span<const double> a, std::span<const double> b) {
    double total = 0.;
    for (std::size_t i = 0; i < a.size(); ++i) total += a[i] * b[i];
    return total;
}

std::vector<double> mat_t_vec(const DenseMatrix &m,
                             std::span<const double> v) {
    std::vector<double> out(m.columns, 0.);
    for (std::size_t r = 0; r < m.rows; ++r)
        for (std::size_t c = 0; c < m.columns; ++c)
            out[c] += m.values[r * m.columns + c] * v[r];
    return out;
}

std::vector<double> mat_vec(const DenseMatrix &m, std::span<const double> v) {
    std::vector<double> out(m.rows, 0.);
    for (std::size_t r = 0; r < m.rows; ++r)
        for (std::size_t c = 0; c < m.columns; ++c)
            out[r] += m.values[r * m.columns + c] * v[c];
    return out;
}

bool isclose(double a, double b, double rtol = 1e-5, double atol = 1e-8) {
    return std::abs(a - b) <= atol + rtol * std::abs(b);
}

void allclose(std::span<const double> a, std::span<const double> b, double rtol,
              double atol, std::string_view what) {
    bt::require(a.size() == b.size(), std::string(what) + ": size");
    for (std::size_t i = 0; i < a.size(); ++i)
        bt::require(std::abs(a[i] - b[i]) <= atol + rtol * std::abs(b[i]),
                    std::string(what) + ": element " + std::to_string(i));
}

std::vector<double> recover(const DenseMatrix &jac,
                            std::span<const double> qfrc) {
    rw::LeastSquaresWorkspace workspace(
        static_cast<rw::lapack_int>(jac.columns),
        static_cast<rw::lapack_int>(jac.rows), 1);
    return rw::recover_wrench(workspace, jac, qfrc);
}

void test_wrench_preserves_virtual_work() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const DenseMatrix jac{3, 4, {1., 0., 0., -1., 0., 1., 0., 0.,
                                 0., 0., 1., 0.}};
    const std::vector<double> expected{20., 100., -2.};
    const std::vector<double> qforce = mat_t_vec(jac, expected);
    const std::vector<double> actual = recover(jac, qforce);
    const std::vector<double> velocity{.2, -.1, .3, .4};
    golden.f64_close(actual, "recovered wrench");
    allclose(actual, expected, 1e-7, 1e-10, "recovered wrench");
    scalar(golden, dot(qforce, velocity), "virtual work");
    bt::require(isclose(dot(actual, mat_vec(jac, velocity)),
                        dot(qforce, velocity)),
                "virtual work preserved");
    golden.done();
}

void test_rank_deficiency_is_not_silently_reported_as_zero_force() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const DenseMatrix jac{2, 4, std::vector<double>(8, 0.)};
    const std::vector<double> qfrc(4, 0.);
    try {
        static_cast<void>(recover(jac, qfrc));
    } catch (const std::exception &error) {
        error_bytes(golden, error);
        golden.done();
        return;
    }
    bt::require(false, "rank-deficient solve must throw");
}

void test_unexplainable_generalized_force_is_not_a_wrench() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const DenseMatrix jac{2, 4, {1., 0., 0., -1., 0., 1., 0., 0.}};
    const std::vector<double> qfrc{1., 0., 0., 1.};
    try {
        static_cast<void>(recover(jac, qfrc));
    } catch (const std::exception &error) {
        error_bytes(golden, error);
        golden.done();
        return;
    }
    bt::require(false, "unexplainable qfrc must throw");
}

constexpr std::string_view kPairXml = R"(
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody>
<equality>{}</equality></mujoco>)";

std::string pair_xml(std::string_view equality) {
    std::string out{kPairXml};
    out.replace(out.find("{}"), 2, equality);
    return out;
}

int body_id(const mjModel *m, const char *name) {
    return mj_name2id(m, mjOBJ_BODY, name);
}

// _recover(model, data, eq_name, rotational) — the (wrench, jac, eq_id)
// triple the source helper returns.
struct Recovered {
    std::vector<double> wrench;
    DenseMatrix jac;
    int eq_id;
    std::vector<double> qfrc_eq;
};

// _lambda: efc_force at the equality's own rows (type==equality AND
// efc_id==eq_id) — the FULL multiplier list, not the lam[:3] surface
// force_on_rider returns.
std::vector<double> full_lambda(const mjData *d,
                                rw::EqualityReactions &reactions,
                                int eq_id) {
    const auto rows = reactions.rows_of(d, eq_id);
    const std::span<const mjtNum> force =
        std::views::counted(d->efc_force, d->nefc);
    std::vector<double> out(rows.size());
    for (std::size_t i = 0; i < rows.size(); ++i)
        out[i] = force[static_cast<std::size_t>(rows[i])];
    return out;
}

Recovered recover_on(const mjModel *m, const mjData *d,
                     rw::EqualityReactions &reactions, const char *eq_name,
                     bool rotational) {
    const int eq_id = mj_name2id(m, mjOBJ_EQUALITY, eq_name);
    bt::require(eq_id >= 0, "equality id resolves");
    const std::vector<double> anchor{0., 0., 1.};
    Recovered out;
    out.eq_id = eq_id;
    out.jac = rw::relative_planar_jacobian(m, d, body_id(m, "rider"),
                                         body_id(m, "bike"), anchor,
                                         rotational);
    out.qfrc_eq.assign(static_cast<std::size_t>(m->nv), 0.);
    reactions.equality_qfrc_into(d, eq_id, out.qfrc_eq);
    out.wrench = recover(out.jac, out.qfrc_eq);
    return out;
}

void weld_case(bt::oracle::Node &golden, const std::array<double, 2> &force) {
    bt::ModelFixture fixture(pair_xml(
        R"(<weld name="w" body1="rider" body2="bike"/>)"));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    const int rider = body_id(m, "rider");
    engine::forward(m, d);
    const std::array<double, 6> xfrc{force[0], 0., force[1], 0., 12.5, 0.};
    const std::span<mjtNum> applied = xfrc_row(m, d, rider);
    for (std::size_t i = 0; i < 6; ++i) applied[i] = xfrc[i];
    for (int step = 0; step < 100; ++step) engine::step(m, d);
    rw::EqualityReactions reactions(m);
    const Recovered r = recover_on(m, d, reactions, "w", true);
    const std::vector<double> lam = full_lambda(d, reactions, r.eq_id);
    // Translational rows are unscaled: recovered force equals multipliers.
    golden.f64_close(std::span<const double>{r.wrench}.first(2),
                     "lam translational");
    allclose(std::span<const double>{r.wrench}.first(2),
             std::vector<double>{lam[0], lam[2]}, 1e-8, 1e-8, "lam");
    // The weld's sin(t/2) residual halves the physical moment in lambda_m.
    scalar(golden, r.wrench[2], "halved moment");
    bt::require(isclose(r.wrench[2], .5 * lam[4], 1e-3, 1e-3),
                "halved moment");
    allclose(r.wrench, std::vector<double>{-force[0], -force[1], -12.5}, .05,
             2., "settled wrench balances applied");
    golden.f64_close(r.qfrc_eq, "qfrc_eq");
    allclose(mat_t_vec(r.jac, r.wrench), r.qfrc_eq, 1e-8, 1e-8, "qfrc_eq");
    const std::span<const double> qvel =
        std::views::counted(d->qvel, m->nv);
    scalar(golden, dot(r.wrench, mat_vec(r.jac, qvel)), "virtual work");
    bt::require(isclose(dot(r.wrench, mat_vec(r.jac, qvel)),
                        dot(r.qfrc_eq, qvel)),
                "virtual work preserved");
    golden.done();
}

void test_weld_recovers_solved_wrench_and_work() {
    {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_weld_recovers_solved_wrench_and_work"
                                "[force0]");
        weld_case(golden, {30., -80.});
    }
    {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_weld_recovers_solved_wrench_and_work"
                                "[force1]");
        weld_case(golden, {-45., 120.});
    }
}

void connect_case(bt::oracle::Node &golden,
                  const std::array<double, 2> &force) {
    bt::ModelFixture fixture(pair_xml(
        R"(<connect name="c" body1="rider" body2="bike" anchor="0 0 0"/>)"));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    const int rider = body_id(m, "rider");
    engine::forward(m, d);
    const std::array<double, 6> xfrc{force[0], 0., force[1], 0., 30., 0.};
    const std::span<mjtNum> applied = xfrc_row(m, d, rider);
    for (std::size_t i = 0; i < 6; ++i) applied[i] = xfrc[i];
    for (int step = 0; step < 100; ++step) engine::step(m, d);
    rw::EqualityReactions reactions(m);
    const Recovered r = recover_on(m, d, reactions, "c", false);
    const std::vector<double> lam = full_lambda(d, reactions, r.eq_id);
    golden.f64_close(r.wrench, "lam");
    allclose(r.wrench, std::vector<double>{lam[0], lam[2]}, 1e-8, 1e-8,
             "lam");
    allclose(r.wrench, std::vector<double>{-force[0], -force[1]}, .05, 2.,
             "pin force balances applied");
    bt::require(r.wrench.size() == 2, "connect wrench is force-only");
    const std::span<const double> qvel =
        std::views::counted(d->qvel, m->nv);
    bt::require(std::abs(qvel[10]) > 0., "rider pitch dof spun up");
    golden.done();
}

void test_connect_recovers_force_only() {
    {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_connect_recovers_force_only[force0]");
        connect_case(golden, {60., 40.});
    }
    {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_connect_recovers_force_only[force1]");
        connect_case(golden, {-25., -95.});
    }
}

void test_contact_rows_do_not_corrupt_equality_selection() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(R"(
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <geom name="floor" type="plane" size="5 5 1" pos="0 0 0"/>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="ball" pos="0.4 0 0.09"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.1"/></body>
</worldbody>
<equality><weld name="w" body1="rider" body2="bike"/></equality></mujoco>)");
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    const int rider = body_id(m, "rider");
    engine::forward(m, d);
    const std::array<double, 6> xfrc{55., 0., 70., 0., 0., 0.};
    const std::span<mjtNum> applied = xfrc_row(m, d, rider);
    for (std::size_t i = 0; i < 6; ++i) applied[i] = xfrc[i];
    for (int step = 0; step < 100; ++step) engine::step(m, d);
    bool non_equality_rows = false;
    const std::span<const int> efc_types =
        std::views::counted(d->efc_type, d->nefc);
    for (std::size_t i = 0; i < efc_types.size(); ++i)
        if (efc_types[i] != mjCNSTR_EQUALITY) non_equality_rows = true;
    bt::require(non_equality_rows, "non-equality rows interleave");
    rw::EqualityReactions reactions(m);
    const Recovered r = recover_on(m, d, reactions, "w", true);
    const std::vector<double> lam = full_lambda(d, reactions, r.eq_id);
    golden.f64_close(std::span<const double>{r.wrench}.first(2),
                     "lam translational");
    allclose(std::span<const double>{r.wrench}.first(2),
             std::vector<double>{lam[0], lam[2]}, 1e-8, 1e-8, "lam");
    allclose(std::span<const double>{r.wrench}.first(2),
             std::vector<double>{-55., -70.}, .05, 2.,
             "wrench balances applied");
    golden.done();
}

void test_prepared_attachment_raw_matches_interval_state_recovery() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(
        R"(<weld name="w" body1="rider" body2="bike"/>)"));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    const int rider = body_id(m, "rider");
    const int bike = body_id(m, "bike");
    engine::forward(m, d);
    const std::span<const mjtNum> qpos_view =
        std::views::counted(d->qpos, m->nq);
    const std::span<const mjtNum> qvel_view =
        std::views::counted(d->qvel, m->nv);
    const std::vector<double> q0(qpos_view.begin(), qpos_view.end());
    const std::vector<double> v0(qvel_view.begin(), qvel_view.end());
    const int eq_id = mj_name2id(m, mjOBJ_EQUALITY, "w");
    const std::vector<double> point{0., 0., 1.};
    const std::vector<double> normal{0., 0., 1.};
    rw::EqualityReactions reactions(m);
    rw::LeastSquaresWorkspace workspace(
        static_cast<rw::lapack_int>(m->nv),
        std::max<rw::lapack_int>(m->nv, 6), 1);
    // rows = {eq_id: flatnonzero(equality rows of eq_id)}
    const rw::EqualityRowMap all_rows = reactions.equality_rows(d);
    rw::EqualityRowMap rows;
    for (const auto &[id, list] : all_rows)
        if (id == eq_id) rows.emplace_back(id, list);
    const rw::AttachmentGeometry geometry = rw::prepare_attachment_geometry(
        m, d, eq_id, rider, bike, point, normal, "foot", true, 0.,
        std::nullopt);
    const std::array<double, 6> xfrc{35., 0., 80., 0., 7., 0.};
    const std::span<mjtNum> applied = xfrc_row(m, d, rider);
    for (std::size_t i = 0; i < 6; ++i) applied[i] = xfrc[i];
    engine::step(m, d);
    std::vector<double> scratch(static_cast<std::size_t>(m->nv), 0.);
    const rw::AttachmentRaw fast = rw::attachment_raw_from_geometry(
        d, reactions, workspace, scratch, geometry, true, &rows);
    const std::span<const mjtNum> post_qpos =
        std::views::counted(d->qpos, m->nq);
    const std::span<const mjtNum> post_qvel =
        std::views::counted(d->qvel, m->nv);
    const std::vector<double> saved_qpos(post_qpos.begin(),
                                         post_qpos.end());
    const std::vector<double> saved_qvel(post_qvel.begin(),
                                         post_qvel.end());
    std::ranges::copy(q0, std::views::counted(d->qpos, m->nq).begin());
    std::ranges::copy(v0, std::views::counted(d->qvel, m->nv).begin());
    engine::kinematics(m, d);
    mj_comPos(m, d);
    const rw::AttachmentRaw slow = rw::attachment_raw(
        m, d, reactions, scratch, eq_id, rider, bike, point, normal, "foot",
        true, 0., std::nullopt, &rows);
    std::ranges::copy(saved_qpos,
        std::views::counted(d->qpos, m->nq).begin());
    std::ranges::copy(saved_qvel,
        std::views::counted(d->qvel, m->nv).begin());
    engine::kinematics(m, d);
    mj_comPos(m, d);
    const std::array<const char *, 4> names{"rider_jac", "bike_jac",
                                            "rider_qfrc", "bike_qfrc"};
    const std::array<std::reference_wrapper<const std::vector<double>>, 4>
        fast_fields{std::cref(fast.rider_jac.values),
                    std::cref(fast.bike_jac.values),
                    std::cref(fast.rider_qfrc),
                    std::cref(fast.bike_qfrc)},
        slow_fields{std::cref(slow.rider_jac.values),
                    std::cref(slow.bike_jac.values),
                    std::cref(slow.rider_qfrc),
                    std::cref(slow.bike_qfrc)};
    for (std::size_t i = 0; i < names.size(); ++i) {
        const char *name = names[i];
        const std::string fast_label = std::string("fast ") + name;
        const std::string slow_label = std::string("slow ") + name;
        golden.f64_close(fast_fields[i].get(), fast_label);
        golden.f64_close(slow_fields[i].get(), slow_label);
        allclose(fast_fields[i].get(), slow_fields[i].get(), 0., 1e-12,
                 name);
    }
    scalar(golden, fast.gap_m, "fast gap_m");
    scalar(golden, slow.gap_m, "slow gap_m");
    bt::require(std::abs(fast.gap_m - slow.gap_m) <= 1e-12,
                "gap matches interval recovery");
    golden.done();
}
} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        test_wrench_preserves_virtual_work();
        test_rank_deficiency_is_not_silently_reported_as_zero_force();
        test_unexplainable_generalized_force_is_not_a_wrench();
        test_weld_recovers_solved_wrench_and_work();
        test_connect_recovers_force_only();
        test_contact_rows_do_not_corrupt_equality_selection();
        test_prepared_attachment_raw_matches_interval_state_recovery();
    });
}
