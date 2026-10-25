// Completion 3.14b (T52). Source: tests/reference/test_native_attachment_wrench.py.
// The T2a/T2b native-oracle seam replayed against the platform goldens:
// equality-row grouping and reaction reads on a settled EFC arena, the
// platform ILP64 DGELSD least-squares contract (output tuple, error and
// FP-flag semantics), recover_wrench, the relative planar jacobian, and
// every prepared/eager attachment measurement face — jacobians, raws and
// budget samples, plus the deferred-validation error paths.
//
// The Python side drives MuJoCo + the binding Stepper side by side and
// records qpos/qvel after every step ('... trajectory diverged'); the
// C++ port owns the same model image, restores the same state/inputs and
// replays the identical steps — every recorded bit stays authoritative.

#include <algorithm>
#include <array>
#include <bit>
#include <cfenv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "engine_call.hpp"
#include "support/golden.hpp"
#include "support/migration_test.hpp"
#include "support/model_fixture.hpp"
#include "rider/attachment_wrench.hpp"
#include "rider/equality_reactions.hpp"
#include "rider/least_squares.hpp"
#include "stepper.hpp"

namespace {
namespace rw = rider;
using rw::DenseMatrix;

constexpr std::string_view kFile =
    "tests/reference/test_native_attachment_wrench.py::";

// ------------------------- golden emit helpers ---------------------------

void scalar(bt::oracle::Node &golden, double actual, std::string_view label) {
    const std::array<double, 1> value{actual};
    golden.f64(value, std::span<const std::size_t>{}, label);
}

void f64_matrix(bt::oracle::Node &golden, const DenseMatrix &matrix,
                std::string_view label) {
    const std::array<std::size_t, 2> shape{matrix.rows, matrix.columns};
    golden.f64(matrix.values, shape, label);
}

void i64_bytes(bt::oracle::Node &golden, std::span<const std::int64_t> values,
               std::string_view label) {
    const std::array<std::size_t, 1> shape{values.size()};
    golden.bytes(std::as_bytes(values), "<i8", shape, label);
}

void str_bytes(bt::oracle::Node &golden, const std::string &text,
               std::string_view label) {
    const std::span<const std::byte> bytes =
        std::as_bytes(std::span{text});
    const std::array<std::size_t, 1> shape{text.size()};
    golden.bytes(bytes, "|u1", shape, label);
}

void error_bytes(bt::oracle::Node &golden, const std::exception &error) {
    str_bytes(golden, error.what(), "error");
}

// Consume the recorded oracle error after proving the port raises the
// same exception text. Returns without consuming when the action does
// not throw — the caller's require marks the run.
template <class Action>
void expect_error(bt::oracle::Node &golden, Action &&action) {
    try {
        std::forward<Action>(action)();
    } catch (const std::exception &error) {
        error_bytes(golden, error);
        return;
    }
    bt::require(false, "expected the recorded ValueError");
}

template <class F> void throws_out_of_range(F &&action) {
    bool threw = false;
    try {
        std::forward<F>(action)();
    } catch (const std::out_of_range &) {
        threw = true;
    }
    bt::require(threw, "expected std::out_of_range (IndexError)");
}

std::string decode_f64_text(const std::vector<double> &encoded) {
    std::string out;
    out.reserve(encoded.size());
    for (const double code : encoded)
        out.push_back(static_cast<char>(static_cast<unsigned char>(code)));
    return out;
}

// Stepper::rider_* attachment faces live in the binding TU
// (rider/attachment_binding.cpp, nanobind) outside the python-free core
// objects. Same forwarding on the Stepper's owned (model, data): shared
// DGELSD workspace bound (nv, max(nv,6), 1) + nv qfrc scratch, matching
// the binding's lazy members exactly.
inline Stepper *st_ptr(const std::unique_ptr<Stepper> &s) {
    return s.get();
}
template <class St>
rw::LeastSquaresWorkspace &lstsq_of(St &&st) {
    static std::map<Stepper *,
                    std::unique_ptr<rw::LeastSquaresWorkspace>> pool;
    Stepper *s = st_ptr(std::forward<St>(st));
    auto &ws = pool[s];
    if (!ws) {
        const auto n = static_cast<rw::lapack_int>(s->model()->nv);
        ws = std::make_unique<rw::LeastSquaresWorkspace>(
            n, std::max<rw::lapack_int>(n, 6), 1);
    }
    return *ws;
}
template <class St> std::vector<double> &scratch_of(St &&st) {
    static std::map<Stepper *, std::vector<double>> pool;
    Stepper *s = st_ptr(std::forward<St>(st));
    auto &v = pool[s];
    const std::size_t nv = static_cast<std::size_t>(s->model()->nv);
    if (v.size() != nv) v.assign(nv, 0.);
    return v;
}

// ---- mjData/mjModel (-Wunsafe-buffer-usage/-Wsign-conversion) --

// d-> ,set_state  const 
std::span<const mjtNum> field(const mjtNum *arr, mjtSize n) {
    return std::views::counted(arr, n);
}
int body_dof(const mjModel *m, int b) {
    return std::views::counted(m->body_dofadr,
        m->nbody)[static_cast<std::size_t>(b)];
}
int jnt_dof(const mjModel *m, int j) {
    return std::views::counted(m->jnt_dofadr,
        m->njnt)[static_cast<std::size_t>(j)];
}
int jnt_posadr(const mjModel *m, int j) {
    return std::views::counted(m->jnt_qposadr,
        m->njnt)[static_cast<std::size_t>(j)];
}
void set_qfrc(const mjModel *m, mjData *d, int dof, mjtNum v) {
    std::views::counted(d->qfrc_applied,
        m->nv)[static_cast<std::size_t>(dof)] = v;
}
void set_qpos(const mjModel *m, mjData *d, int idx, mjtNum v) {
    std::views::counted(d->qpos,
        m->nq)[static_cast<std::size_t>(idx)] = v;
}

// --------------------------- model fixtures ------------------------------

std::vector<std::byte> save_mjb(const mjModel *model) {
    engine::ErrorBuffer error;
    struct SizeCall {
        const mjModel *model;
        int size;
    } call{.model = model, .size = 0};
    if (!engine::invoke(
            [](void *raw) noexcept {
                auto &c = *static_cast<SizeCall *>(raw);
                c.size = static_cast<int>(mj_sizeModel(c.model));
            },
            &call, error))
        engine::throw_failure(error);
    std::vector<std::byte> image(static_cast<std::size_t>(call.size));
    struct SaveCall {
        const mjModel *model;
        void *bytes;
        int size;
    } save{.model = model, .bytes = image.data(), .size = call.size};
    if (!engine::invoke(
            [](void *raw) noexcept {
                const auto &c = *static_cast<SaveCall *>(raw);
                mj_saveModel(c.model, nullptr, c.bytes, c.size);
            },
            &save, error))
        engine::throw_failure(error);
    return image;
}

constexpr std::string_view kPairPrefix = R"(
<mujoco><option gravity="0 0 0" timestep="0.0005" )";
constexpr std::string_view kPairBodies = R"(/>
<worldbody>
  <geom name="floor" type="plane" size="5 5 1" pos="0 0 0"/>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="hand" pos=".2 0 1.1"><freejoint/>
    <inertial pos="0 0 0" mass=".5" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="ja_b" pos=".5 0 1"><joint name="ja" type="hinge" axis="0 1 0"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="jb_b" pos=".7 0 1"><joint name="jb" type="hinge" axis="0 1 0"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="lim_b" pos="-.5 0 1">
    <joint name="lim" type="hinge" axis="0 1 0" range="-0.02 0.02"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="ball" pos="0.4 0 0.09"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.1"/></body>
</worldbody>)";

// _pair_model(equality_xml, option=...) — the small rider/bike/hand pair
// with unrelated contact and limit rows interleaving the arena.
std::string pair_xml(std::string_view equality, std::string_view option = "") {
    std::string out{kPairPrefix};
    out += option;
    out += kPairBodies;
    if (!equality.empty()) {
        out += "<equality>";
        out += equality;
        out += "</equality>";
    }
    out += "</mujoco>";
    return out;
}

constexpr std::string_view kAllEqualities =
    R"(<weld name="w" body1="rider" body2="bike"/>)"
    R"(<joint name="j" joint1="ja" joint2="jb"/>)"
    R"(<connect name="c" body1="hand" body2="bike" anchor="0 .1 0"/>)";
constexpr std::string_view kWeld =
    R"(<weld name="w" body1="rider" body2="bike"/>)";

int body(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_BODY, name);
    bt::require(id >= 0, std::string("body resolves: ") + name);
    return id;
}

int equality(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_EQUALITY, name);
    bt::require(id >= 0, std::string("equality resolves: ") + name);
    return id;
}

int joint(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_JOINT, name);
    bt::require(id >= 0, std::string("joint resolves: ") + name);
    return id;
}

// _drive: break the 'lim' hinge range, load rider/ja/hand so every
// equality in the arena carries a real multiplier.
void drive(const mjModel *m, mjData *d,
           const std::array<double, 3> &force = {30., -80., 12.5}) {
    set_qpos(m, d, jnt_posadr(m, joint(m, "lim")), .4);
    engine::forward(m, d);
    const int rider_dof = body_dof(m, body(m, "rider"));
    set_qfrc(m, d, rider_dof, force[0]);
    set_qfrc(m, d, rider_dof + 2, force[1]);
    set_qfrc(m, d, rider_dof + 4, force[2]);
    set_qfrc(m, d, jnt_dof(m, joint(m, "ja")), 6.5);
    const int hand_dof = body_dof(m, body(m, "hand"));
    set_qfrc(m, d, hand_dof, 9.);
    set_qfrc(m, d, hand_dof + 1, -4.5);
}

// _native_mirror: a Stepper over the same model bytes replaying the same
// solver inputs (state + ctrl/qfrc_applied) — no second simulation.
std::unique_ptr<Stepper> mirror(const mjModel *m, const mjData *d) {
    const std::vector<std::byte> image = save_mjb(m);
    auto st = std::make_unique<Stepper>(std::span<const std::byte>(image));
    st->set_state(field(d->qpos, m->nq), field(d->qvel, m->nv),
                  field(d->act, m->na),
                  field(d->qacc_warmstart, m->nv), d->time);
    st->set_inputs(std::views::counted(d->ctrl, m->nu),
                   std::views::counted(d->qfrc_applied, m->nv));
    return st;
}

void feed_inputs(const mjModel *m, mjData *d, Stepper &st) {
    st.set_inputs(std::views::counted(d->ctrl, m->nu),
                  std::views::counted(d->qfrc_applied, m->nv));
}

// _settle: replay the recorded steps; the trailing 'qpos/qvel trajectory
// diverged' pair decides the count, so each node replays exactly its
// recorded number of steps (100 default, 40 for the grouping node).
void settle(const mjModel *m, mjData *d, Stepper &st,
            bt::oracle::Node &golden) {
    while (golden.peek_label() == "qpos trajectory diverged") {
        // _settle steps BOTH arenas — the fixture mjData mirrors the
        // recorded trajectory so post-settle data consumers (jac reads,
        // raw samples) see the same state the stepper holds.
        engine::step(m, d);
        st.step();
        golden.f64(st.qpos(), "qpos trajectory diverged");
        golden.f64(st.qvel(), "qvel trajectory diverged");
    }
}

const std::vector<double> kPoint{0., 0., 1.};
const std::vector<double> kNormal{0., 0., 1.};

// -------------------- _assert_geometry / _assert_raw ----------------------
// Recorded field order mirrors the source helpers verbatim.

void assert_geometry(bt::oracle::Node &golden,
                     const rw::AttachmentGeometry &g,
                     const std::string &msg) {
    scalar(golden, static_cast<double>(g.eq_id), msg + " eq_id");
    f64_matrix(golden, g.rider_jac, msg + " rider_jac");
    i64_bytes(golden, g.rider_columns, msg + " rider_columns");
    f64_matrix(golden, g.bike_jac, msg + " bike_jac");
    i64_bytes(golden, g.bike_columns, msg + " bike_columns");
    scalar(golden, g.observable ? 1. : 0., msg + " observable");
    golden.f64(g.normal, msg + " normal");
    str_bytes(golden, g.kind, msg + " kind");
    scalar(golden, g.rotational ? 1. : 0., msg + " rotational");
    scalar(golden, g.half_patch_m, msg + " half_patch_m");
    scalar(golden, g.pull_direction.has_value() ? 1. : 0.,
           msg + " pull present");
    if (g.pull_direction.has_value())
        golden.f64(*g.pull_direction, msg + " pull_direction");
}

void assert_raw(bt::oracle::Node &golden, const rw::AttachmentRaw &r,
                const std::string &msg) {
    f64_matrix(golden, r.rider_jac, msg + " rider_jac");
    golden.f64(r.rider_qfrc, msg + " rider_qfrc");
    f64_matrix(golden, r.bike_jac, msg + " bike_jac");
    golden.f64(r.bike_qfrc, msg + " bike_qfrc");
    scalar(golden, r.observable ? 1. : 0., msg + " observable");
    golden.f64(r.normal, msg + " normal");
    str_bytes(golden, r.kind, msg + " kind");
    scalar(golden, r.rotational ? 1. : 0., msg + " rotational");
    scalar(golden, r.half_patch_m, msg + " half_patch_m");
    scalar(golden, r.gap_m, msg + " gap_m");
    scalar(golden, r.pull_direction.has_value() ? 1. : 0.,
           msg + " pull present");
    if (r.pull_direction.has_value())
        golden.f64(*r.pull_direction, msg + " pull_direction");
}

// _assert_sample — the AttachmentSample dataclass's declaration order:
// kind then the six numeric fields.
void assert_sample(bt::oracle::Node &golden,
                   const rw::AttachmentSample &s,
                   const std::string &msg) {
    str_bytes(golden, s.kind, msg + " kind");
    scalar(golden, s.normal_n, msg + " normal_n");
    scalar(golden, s.tangent_n, msg + " tangent_n");
    scalar(golden, s.moment_nm, msg + " moment_nm");
    scalar(golden, s.gap_m, msg + " gap_m");
    scalar(golden, s.pull_n, msg + " pull_n");
    scalar(golden, s.half_patch_m, msg + " half_patch_m");
}

// ---------------------- equality reads (T2a) ------------------------------

// test_rider_equality_qfrc_oracle[sparse|dense]
void equality_qfrc_case(const char *jacobian) {
    const std::string node = std::string(kFile) +
        "test_rider_equality_qfrc_oracle[" + jacobian + "]";
    bt::oracle::Node golden(node);
    bt::ModelFixture fixture(
        pair_xml(kAllEqualities, std::string("jacobian=\"") + jacobian + "\""));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const mjData *sd = st->data();
    bool saw_non_equality = false, saw_limit = false;
    const std::span<const int> efc_types =
        std::views::counted(sd->efc_type, sd->nefc);
    for (std::size_t i = 0; i < efc_types.size(); ++i) {
        saw_non_equality = saw_non_equality ||
            efc_types[i] != mjCNSTR_EQUALITY;
        saw_limit = saw_limit || efc_types[i] == mjCNSTR_LIMIT_JOINT;
    }
    bt::require(saw_non_equality, "unrelated rows really present");
    bt::require(saw_limit, "a violated joint limit row is present");
    rw::EqualityReactions &reactions = st->rider_equalities();
    for (const char *name : {"w", "j", "c"}) {
        const int eq_id = equality(m, name);
        golden.f64(st->rider_equality_qfrc(eq_id),
                   std::string("qfrc equality ") + name);
        const std::vector<double> lam =
            reactions.force_on_rider(sd, eq_id);
        bt::require(!lam.empty() &&
                        std::ranges::any_of(lam, [](double v) {
                            return v != 0.;
                        }),
                    "the equality carries a real load");
    }
    // A bogus id selects nothing — membership is re-derived per call.
    golden.f64(st->rider_equality_qfrc(999), "");
    golden.f64(st->rider_equality_qfrc(999), "");
    // A limit row's efc_id is not an equality id — the type half of the
    // mask excludes it even when the ids collide.
    const auto limit_id = static_cast<int>(
        golden.f64_input("input.limit_id").at(0));
    golden.f64(st->rider_equality_qfrc(limit_id), "");
    golden.done();
}

void test_rider_equality_qfrc_oracle() {
    equality_qfrc_case("sparse");
    equality_qfrc_case("dense");
}

void test_rider_equality_rows_grouping() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kAllEqualities));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const rw::EqualityRowMap rows =
        st->rider_equalities().equality_rows(st->data());
    bt::require(rows.size() == 3, "three equality groups");
    std::vector<std::int64_t> ids;
    ids.reserve(rows.size());
    for (const auto &[eq_id, list] : rows) {
        ids.push_back(eq_id);
        bt::require(
            eq_id == equality(m, "w") || eq_id == equality(m, "j") ||
                eq_id == equality(m, "c"),
            "only the declared equalities group");
    }
    i64_bytes(golden, ids, "equality ids");
    for (const auto &[eq_id, list] : rows) i64_bytes(golden, list, "");
    golden.done();
}

// test_rider_equality_reaction_oracle[w|j|c]: rows (i8), force_on_rider,
// translation_residual, qfrc — all bitwise on the shared arena.
void equality_reaction_case(const char *eq_name) {
    const std::string node = std::string(kFile) +
        "test_rider_equality_reaction_oracle[" + eq_name + "]";
    bt::oracle::Node golden(node);
    bt::ModelFixture fixture(pair_xml(kAllEqualities));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int eq_id = equality(m, eq_name);
    const rw::EqualityReactions &reactions = st->rider_equalities();
    i64_bytes(golden, reactions.rows_of(st->data(), eq_id), "");
    golden.f64(reactions.force_on_rider(st->data(), eq_id), "");
    scalar(golden, reactions.translation_residual(st->data(), eq_id), "");
    golden.f64(st->rider_equality_qfrc(eq_id), "");
    golden.done();
}

void test_rider_equality_reaction_oracle() {
    for (const char *name : {"w", "j", "c"}) equality_reaction_case(name);
}

void test_rider_equality_reaction_missing_rows_zero() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kWeld));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const rw::EqualityReactions &reactions = st->rider_equalities();
    i64_bytes(golden, reactions.rows_of(st->data(), 77), "");
    golden.f64(reactions.force_on_rider(st->data(), 77), "");
    scalar(golden, reactions.translation_residual(st->data(), 77),
           "translation_residual_m");
    golden.f64(st->rider_equality_qfrc(77), "");
    golden.done();
}

void test_rider_equality_qfrc_empty_arena() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(R"(<mujoco><worldbody>
  <body pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody></mujoco>)");
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    engine::forward(m, d);
    bt::require(d->nefc == 0, "empty constraint arena");
    const auto st = mirror(m, d);
    // nefc == 0: mj_mulJacTVec's early return must not leak an
    // uninitialized output — zeros(nv) on both reads.
    golden.f64(st->rider_equality_qfrc(0), "");
    golden.f64(st->rider_equality_qfrc(0), "");
    const std::array<int, 1> ids{0};
    scalar(golden,
           st->rider_equalities().equalities_qfrc_at(st->data(), ids, 0),
           "equalities torque");
    golden.done();
}

void test_rider_pedal_torque_oracle() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(R"(
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/>
    <body name="crank" pos="0 .2 0">
      <joint name="crank_spin" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="5" diaginertia="1 1 1"/>
      <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  </body>
  <body name="foot_a" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="foot_b" pos="0 .4 1"><freejoint/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody>
<equality>
  <weld name="weld_foot_front" body1="foot_a" body2="crank"/>
  <weld name="weld_foot_rear" body1="foot_b" body2="bike"/>
</equality></mujoco>)");
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    engine::forward(m, d);
    const int foot_a = body(m, "foot_a");
    const int foot_b = body(m, "foot_b");
    set_qfrc(m, d, body_dof(m, foot_a) + 0, 22.);
    set_qfrc(m, d, body_dof(m, foot_a) + 2, -55.);
    set_qfrc(m, d, body_dof(m, foot_b) + 0, -11.);
    set_qfrc(m, d, body_dof(m, foot_b) + 2, -37.);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    // PedalWelds' 'weld_foot_{front,rear}' + 'crank_spin' contract.
    std::vector<int> eq_ids{equality(m, "weld_foot_front"),
                            equality(m, "weld_foot_rear")};
    std::ranges::sort(eq_ids);
    const int crank_dof = jnt_dof(m, joint(m, "crank_spin"));
    scalar(golden,
           st->rider_equalities().equalities_qfrc_at(st->data(), eq_ids,
                                                   crank_dof),
           "");
    // A nonexistent equality contributes nothing — same as a zero mask.
    const std::array<int, 2> missing{eq_ids[0], 999};
    scalar(golden,
           st->rider_equalities().equalities_qfrc_at(st->data(), missing,
                                                   crank_dof),
           "");
    golden.done();
}

void test_stepper_qfrc_returns_owning_array() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kWeld));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int eq_id = equality(m, "w");
    std::vector<double> first = st->rider_equality_qfrc(eq_id);
    const std::vector<double> second = st->rider_equality_qfrc(eq_id);
    bt::require(first.data() != second.data(),
                "each call owns a fresh buffer");
    std::ranges::fill(first, 999.);
    golden.f64(second, "");
    golden.done();
}

// ------------------------------ DGELSD lstsq ------------------------------

struct LstsqInput {
    std::vector<double> a, b;
    std::vector<std::size_t> a_shape, b_shape;
    double rcond = 0.;
};

std::vector<std::size_t> dims(const std::vector<double> &recorded) {
    std::vector<std::size_t> out;
    out.reserve(recorded.size());
    for (const double v : recorded) {
        const auto d = static_cast<std::size_t>(v);
        bt::require(static_cast<double>(d) == v,
                    "recorded dims are integral");
        out.push_back(d);
    }
    return out;
}

// least_squares_hook's typed core: the same shape checks the binding
// performs ('expected a 2-dimensional array for 'a'' / 'Incompatible
// dimensions' / 'expected a 1- or 2-dimensional array for 'b''), the
// n_rhs == 0 padding quirk, and the owning result — minus the ndarray
// casts, which have no typed-face counterpart.
rw::LeastSquaresResult lstsq_solve(const LstsqInput &in) {
    if (in.a_shape.size() != 2)
        throw std::invalid_argument(
            "expected a 2-dimensional array for 'a'");
    const std::size_t m = in.a_shape[0], n = in.a_shape[1];
    bt::require(in.a.size() == m * n, "input.a matches its shape");
    std::size_t ek = 0;
    if (in.b_shape.size() == 1) {
        if (in.b_shape[0] != m)
            throw std::invalid_argument("Incompatible dimensions");
        ek = 1;
    } else if (in.b_shape.size() == 2) {
        if (in.b_shape[0] != m)
            throw std::invalid_argument("Incompatible dimensions");
        ek = in.b_shape[1];
    } else {
        throw std::invalid_argument(
            "expected a 1- or 2-dimensional array for 'b'");
    }
    bt::require(in.b.size() == m * ek ||
                    (in.b_shape.size() == 1 && in.b.size() == m),
                "input.b matches its shape");
    std::vector<double> zero_rhs;
    std::span<const double> b_span = in.b;
    std::size_t solve_rhs = ek;
    if (ek == 0) {
        zero_rhs.assign(m, 0.);
        b_span = zero_rhs;
        solve_rhs = 1;
    }
    rw::LeastSquaresWorkspace workspace(
        static_cast<rw::lapack_int>(m), static_cast<rw::lapack_int>(n),
        static_cast<rw::lapack_int>(solve_rhs));
    rw::LeastSquaresResult out =
        workspace.solve(in.a, static_cast<rw::lapack_int>(m),
                        static_cast<rw::lapack_int>(n), b_span,
                        static_cast<rw::lapack_int>(solve_rhs), in.rcond);
    if (ek == 0) {
        out.solution.clear();
        out.residuals.clear();
    }
    return out;
}

LstsqInput lstsq_input(bt::oracle::Node &golden) {
    LstsqInput in;
    in.a = golden.f64_input("input.a");
    in.a_shape = dims(golden.f64_input("input.a.shape"));
    in.b = golden.f64_input("input.b");
    in.b_shape = dims(golden.f64_input("input.b.shape"));
    in.rcond = golden.f64_input("input.rcond").at(0);
    return in;
}

// Emit the recorded outputs of one staged call. The tuple order is fixed
// (x, resids, rank, singular values); the source records a subset — an
// '' label resolves to the next tuple member in order. An 'error' entry
// expects the same failure text.
template <class Solver>
void lstsq_emit(bt::oracle::Node &golden, const LstsqInput &in,
                Solver &&solver) {
    if (golden.peek_label() == "error") {
        expect_error(golden, [&] { static_cast<void>(solver(in)); });
        return;
    }
    const rw::LeastSquaresResult out = solver(in);
    int member = 0;
    for (;;) {
        const std::string_view label = golden.peek_label();
        int which = -1;
        if (label == "x")
            which = 0;
        else if (label == "resids")
            which = 1;
        else if (label == "rank")
            which = 2;
        else if (label == "singular values")
            which = 3;
        else if (label.empty() && member <= 3 && golden.has_next())
            which = member;
        else
            break;
        bt::require(which >= member, "lstsq outputs stay in tuple order");
        member = which + 1;
        switch (which) {
        case 0:
            golden.f64(out.solution, golden.peek_shape(), label);
            break;
        case 1:
            golden.f64(out.residuals, golden.peek_shape(), label);
            break;
        case 2:
            scalar(golden, static_cast<double>(out.rank), label);
            break;
        default:
            golden.f64(out.singular_values, golden.peek_shape(), label);
            break;
        }
    }
}

void lstsq_call(bt::oracle::Node &golden) {
    lstsq_emit(golden, lstsq_input(golden),
               [](const LstsqInput &in) { return lstsq_solve(in); });
}

void lstsq_calls(bt::oracle::Node &golden) {
    while (golden.peek_label() == "input.a") lstsq_call(golden);
}

// Every parametrized lstsq oracle node is a pure replay loop: the
// recorded (a, b, rcond) triples decide the count.
void test_least_squares_oracle() {
    for (const char *id :
         {"8-3-1", "3-8-2", "5-5-3", "10-4-4", "4-10-5",
          "1-1-6", "7-7-7", "2-9-8", "9-2-9"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_least_squares_oracle[" + id + "]");
        lstsq_calls(golden);
        golden.done();
    }
}

void test_least_squares_rhs_count() {
    for (const char *id : {"0", "1", "2", "5"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_least_squares_rhs_count[" + id + "]");
        lstsq_calls(golden);
        golden.done();
    }
}

void test_least_squares_degenerate_dims() {
    for (const char *id : {"degenerate0", "degenerate1", "degenerate2",
                           "degenerate3"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_least_squares_degenerate_dims[" + id +
                                "]");
        lstsq_calls(golden);
        golden.done();
    }
}

void test_least_squares_rank_deficient() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    lstsq_calls(golden);
    golden.done();
}

void test_least_squares_rcond_boundary() {
    for (const char *id : {"0.9", "0.99", "1.0", "1.01", "1.1"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_least_squares_rcond_boundary[" + id +
                                "]");
        lstsq_calls(golden);
        golden.done();
    }
}

void test_least_squares_rcond_values() {
    for (const char *id : {"1e-12", "-1.0", "0.0", "0.5", "0.001"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_least_squares_rcond_values[" + id +
                                "]");
        lstsq_calls(golden);
        golden.done();
    }
}

void test_least_squares_inconsistent_target() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    lstsq_calls(golden);
    golden.done();
}

void test_least_squares_wide_minimum_norm() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    lstsq_calls(golden);
    golden.done();
}

void test_least_squares_nonfinite() {
    for (const char *id : {"a0-b0", "a1-b1", "a2-b2", "a3-b3", "a4-b4",
                           "a5-b5"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_least_squares_nonfinite[" + id + "]");
        lstsq_calls(golden);
        golden.done();
    }
}

void test_least_squares_shape_errors() {
    // No golden node: the raise-only shape contract. 'Incompatible
    // dimensions' for a mismatched b row count; the ndarray-rank wording
    // ('expected a 2-dimensional array for 'a'' / 'expected a 1- or
    // 2-dimensional array for 'b'') for wrong-rank operands.
    const std::vector<double> eye3{1., 0., 0., 0., 1., 0., 0., 0., 1.};
    const std::vector<double> zeros4(4, 0.);
    bt::invalid([&] {
        static_cast<void>(lstsq_solve(
            {.a = eye3, .b = zeros4, .a_shape = {3, 3},
             .b_shape = {4}, .rcond = 1e-12}));
    });
    bt::invalid([&] {
        static_cast<void>(lstsq_solve(
            {.a = zeros4, .b = zeros4, .a_shape = {4}, .b_shape = {4},
             .rcond = 1e-12}));
    });
    bt::invalid([&] {
        static_cast<void>(lstsq_solve(
            {.a = zeros4, .b = {0.}, .a_shape = {2, 2}, .b_shape = {},
             .rcond = 1e-12}));
    });
    bt::invalid([&] {
        static_cast<void>(lstsq_solve(
            {.a = eye3, .b = {0., 0., 0.}, .a_shape = {3, 3},
             .b_shape = {3, 1, 1}, .rcond = 1e-12}));
    });
    bt::invalid([&] {
        static_cast<void>(lstsq_solve(
            {.a = zeros4, .b = {0., 0., 0.}, .a_shape = {3, 2, 2},
             .b_shape = {3}, .rcond = 1e-12}));
    });
}

void test_least_squares_incompatible_dimensions_message() {
    // Same analytic contract — assert the numpy wording reaches the
    // caller verbatim.
    bool matched = false;
    try {
        static_cast<void>(lstsq_solve(
            {.a = {1., 0., 0., 0., 1., 0., 0., 0., 1.}, .b = {0., 0., 0., 0.},
             .a_shape = {3, 3}, .b_shape = {4}, .rcond = 1e-12}));
    } catch (const std::invalid_argument &error) {
        matched = std::string_view(error.what()).find(
                      "Incompatible dimensions") != std::string_view::npos;
    }
    bt::require(matched, "lstsq dimension error names 'Incompatible dimensions'");
}

void test_least_squares_output_layout_and_ownership() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    lstsq_call(golden);  // f32-cast inputs: x only
    lstsq_call(golden);  // non-contiguous inputs: the full tuple
    // Fresh owned storage on every call: mutating one result disturbs no
    // other, and outputs never alias the input buffers.
    lstsq_emit(
        golden, lstsq_input(golden), [](const LstsqInput &in) {
            rw::LeastSquaresResult first = lstsq_solve(in);
            rw::LeastSquaresResult second = lstsq_solve(in);
            bt::require(first.solution.data() != second.solution.data(),
                        "solution buffers are unshared");
            bt::require(
                first.singular_values.data() != second.singular_values.data(),
                "singular-value buffers are unshared");
            bt::require(first.solution.data() != in.a.data() &&
                            first.solution.data() != in.b.data(),
                        "outputs never alias the inputs");
            std::ranges::fill(first.solution, 0.);
            return second;
        });
    golden.done();
}

void test_linux_and_mac_lstsq_use_numpy_workspace_contract() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    lstsq_calls(golden);
    golden.done();
}

void test_least_squares_prior_fp_invalid_flag() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const LstsqInput in = lstsq_input(golden);
    // FE_INVALID on entry is the gufunc's error_occurred seed: it never
    // fails a well-posed solve and the flag survives the call.
    std::feclearexcept(FE_ALL_EXCEPT);
    std::feraiseexcept(FE_INVALID);
    lstsq_emit(golden, in,
               [](const LstsqInput &staged) { return lstsq_solve(staged); });
    bt::require(std::fetestexcept(FE_INVALID) != 0,
                "the entry flag is preserved through a successful solve");
    // Clean entry: a successful solve leaves every flag cleared.
    std::feclearexcept(FE_ALL_EXCEPT);
    static_cast<void>(lstsq_solve(in));
    bt::require(std::fetestexcept(FE_ALL_EXCEPT) == 0,
                "a clean solve leaves all FP flags clear");
    std::feclearexcept(FE_ALL_EXCEPT);
    golden.done();
}

void test_least_squares_fp_invalid_raised_on_failure() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const LstsqInput in = lstsq_input(golden);
    // The failure path raises the flag like the oracle's failed lstsq —
    // including when the flag was already set on entry.
    std::feclearexcept(FE_ALL_EXCEPT);
    std::feraiseexcept(FE_INVALID);
    lstsq_emit(golden, in,
               [](const LstsqInput &staged) { return lstsq_solve(staged); });
    bt::require(std::fetestexcept(FE_INVALID) != 0,
                "a non-convergent solve leaves FE_INVALID set");
    std::feclearexcept(FE_ALL_EXCEPT);
    golden.done();
}

// ----------------------------- recover_wrench ---------------------------

std::vector<double> recover(const DenseMatrix &jac,
                            std::span<const double> qfrc) {
    rw::LeastSquaresWorkspace workspace(
        static_cast<rw::lapack_int>(jac.columns),
        static_cast<rw::lapack_int>(jac.rows), 1);
    return rw::recover_wrench(workspace, jac, qfrc);
}

void recover_call(bt::oracle::Node &golden) {
    const std::vector<double> flat = golden.f64_input("input.jac");
    const std::vector<std::size_t> shape =
        dims(golden.f64_input("input.jac.shape"));
    const std::vector<double> qfrc = golden.f64_input("input.qfrc");
    bt::require(shape.size() == 2 && flat.size() == shape[0] * shape[1],
                "input.jac matches its shape");
    const DenseMatrix jac{.rows = shape[0], .columns = shape[1],
                          .values = flat};
    if (golden.peek_label() == "error") {
        expect_error(golden, [&] { static_cast<void>(recover(jac, qfrc)); });
        return;
    }
    golden.f64(recover(jac, qfrc), golden.peek_shape(), "");
}

void recover_calls(bt::oracle::Node &golden) {
    while (golden.peek_label() == "input.jac") recover_call(golden);
}

void test_recover_wrench_random_oracle() {
    for (const char *id :
         {"shape0-1", "shape1-2", "shape2-3", "shape3-4", "shape4-5",
          "shape5-6", "shape6-7", "shape7-8", "shape8-9"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_recover_wrench_random_oracle[" + id +
                                "]");
        recover_calls(golden);
        golden.done();
    }
}

void test_recover_wrench_rank_deficient_and_inconsistent() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    recover_calls(golden);
    golden.done();
}

void test_recover_wrench_rcond_boundary() {
    for (const char *id : {"0.9", "0.99", "1.0", "1.01", "1.1"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_recover_wrench_rcond_boundary[" + id +
                                "]");
        recover_calls(golden);
        golden.done();
    }
}

void test_recover_wrench_nonfinite() {
    for (const char *id : {"jac0-qfrc0", "jac1-qfrc1", "jac2-qfrc2",
                           "jac3-qfrc3"}) {
        bt::oracle::Node golden(std::string(kFile) +
                                "test_recover_wrench_nonfinite[" + id + "]");
        recover_calls(golden);
        golden.done();
    }
}

void test_recover_wrench_degenerate_and_shape_errors() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    recover_calls(golden);
    // The remaining raise-only contracts: a 1-D or 3-D jacobian is a
    // binding-cast rejection with no typed-face counterpart (documented
    // drop); a qfrc width that disagrees with the jacobian's columns is
    // the same ValueError on this face.
    const DenseMatrix eye3{3, 3, {1., 0., 0., 0., 1., 0., 0., 0., 1.}};
    const std::vector<double> too_wide(4, 0.);
    bt::invalid([&] { static_cast<void>(recover(eye3, too_wide)); });
    golden.done();
}

void test_recover_wrench_owns_its_output() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const std::vector<double> flat = golden.f64_input("input.jac");
    const std::vector<std::size_t> shape =
        dims(golden.f64_input("input.jac.shape"));
    const std::vector<double> qfrc = golden.f64_input("input.qfrc");
    bt::require(shape.size() == 2 && flat.size() == shape[0] * shape[1],
                "input.jac matches its shape");
    const DenseMatrix jac{.rows = shape[0], .columns = shape[1],
                          .values = flat};
    std::vector<double> first = recover(jac, qfrc);
    const std::vector<double> second = recover(jac, qfrc);
    bt::require(first.data() != second.data() &&
                    first.data() != jac.values.data(),
                "fresh, unshared, owning output storage");
    std::ranges::fill(first, -1.);
    golden.f64(second, golden.peek_shape(), "");
    golden.done();
}

void test_native_recover_wrench() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    const DenseMatrix jac{3, 4, {1., 0., 0., -1., 0., 1., 0., 0.,
                                 0., 0., 1., 0.}};
    // force = jac.T @ [20, 100, -2] — ddot order per column, like the
    // oracle's matmul on the transpose view.
    std::vector<double> force(jac.columns, 0.);
    const std::array<double, 3> w{20., 100., -2.};
    for (std::size_t r = 0; r < jac.rows; ++r)
        for (std::size_t c = 0; c < jac.columns; ++c)
            force[c] += jac.values[r * jac.columns + c] * w[r];
    golden.f64(recover(jac, force), golden.peek_shape(), "");
    golden.done();
}

// --------------------- relative planar jacobian --------------------------

void planar_jacobian_case(const char *jacobian) {
    const std::string node = std::string(kFile) +
        "test_relative_planar_jacobian_oracle[" + jacobian + "]";
    bt::oracle::Node golden(node);
    bt::ModelFixture fixture(
        pair_xml(kAllEqualities, std::string("jacobian=\"") + jacobian + "\""));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int rider = body(m, "rider"), bike = body(m, "bike");
    const int hand = body(m, "hand");
    f64_matrix(golden,
               rw::relative_planar_jacobian(st->model(), st->data(), rider, bike, kPoint,
                                                  false),
               "rotational=False");
    f64_matrix(golden,
               rw::relative_planar_jacobian(st->model(), st->data(), rider, bike, kPoint,
                                                  true),
               "rotational=True");
    // A moving point reads the same engine jacobian: mj_jac reads cdof,
    // refreshed only by mj_forward — mirror the oracle's forward.
    set_qpos(m, d, 0, .03);
    set_qpos(m, d, 1, .0);
    set_qpos(m, d, 2, -.02);
    set_qpos(m, d, 3, .965925826);
    engine::forward(m, d);
    st->set_state(field(d->qpos, m->nq), field(d->qvel, m->nv),
                  field(d->act, m->na),
                  field(d->qacc_warmstart, m->nv), d->time);
    st->forward();
    const std::vector<double> moved{.4, 0., .9};
    f64_matrix(golden,
               rw::relative_planar_jacobian(st->model(), st->data(), rider, bike, moved, true),
               "");
    f64_matrix(golden,
               rw::relative_planar_jacobian(st->model(), st->data(), hand, bike, moved, false),
               "");
    // 'invalid world point' / 'distinct physical bodies' — same text.
    const std::vector<double> short_point{0., 0.};
    const std::vector<double> nan_point{0., 0.,
                                        std::numeric_limits<double>::quiet_NaN()};
    const std::vector<double> inf_point{0., 0.,
                                        std::numeric_limits<double>::infinity()};
    for (const std::vector<double> *bad :
         {&short_point, &nan_point, &inf_point})
        expect_error(golden, [&] {
            static_cast<void>(rw::relative_planar_jacobian(st->model(), st->data(), 
                rider, bike, *bad, true));
        });
    const int nbody = static_cast<int>(m->nbody);
    for (const std::pair<int, int> &bad :
         std::array{std::pair{0, bike}, std::pair{rider, 0},
                    std::pair{rider, rider}, std::pair{rider, nbody}})
        expect_error(golden, [&] {
            static_cast<void>(rw::relative_planar_jacobian(st->model(), st->data(), 
                bad.first, bad.second, moved, true));
        });
    golden.done();
}

void test_relative_planar_jacobian_oracle() {
    planar_jacobian_case("sparse");
    planar_jacobian_case("dense");
}

// ---------------------- T2b: solved attachment measurements ---------------

// test_prepare_attachment_geometry_oracle — every AttachmentGeometry
// field bitwise, then the 'no degrees of freedom' body gate.
void prepare_geometry_case(const char *id, const char *jacobian,
                           const char *eq_name, const char *body_a,
                           const char *body_b, const std::vector<double> &point,
                           bool rotational, const char *kind,
                           const std::optional<std::vector<double>> &pull) {
    const std::string node = std::string(kFile) +
        "test_prepare_attachment_geometry_oracle[" + id + "]";
    bt::oracle::Node golden(node);
    bt::ModelFixture fixture(
        pair_xml(R"(<weld name="w" body1="rider" body2="bike"/>)"
                 R"(<connect name="c" body1="hand" body2="bike" anchor="0 .1 0"/>)",
                 std::string("jacobian=\"") + jacobian + "\""));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int eq_id = equality(m, eq_name);
    const int a = body(m, body_a), b = body(m, body_b);
    const rw::AttachmentGeometry actual = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, a, b, point, kNormal, kind, rotational, .05, pull);
    assert_geometry(golden, actual, "");
    // A body carrying no dofs is a hard failure on both sides.
    expect_error(golden, [&] {
        static_cast<void>(rw::prepare_attachment_geometry(st->model(), st->data(), 
            eq_id, 0, b, point, kNormal, kind, rotational, 0.,
            std::nullopt));
    });
    golden.done();
}

void test_prepare_attachment_geometry_oracle() {
    // Stacked parametrize ids: the inner (eq_name,bodies,point,rotational,
    // kind,pull) tuple first, then the jacobian — 'w-bodies0-point0-
    // True-foot-pull0-sparse' etc.
    for (const char *jacobian : {"sparse", "dense"}) {
        const std::string suffix = std::string("-") + jacobian;
        prepare_geometry_case(
            ("w-bodies0-point0-True-foot-pull0" + suffix).c_str(), jacobian,
            "w", "rider", "bike", kPoint, true, "foot",
            std::vector<double>{.1, 0., -.8});
        prepare_geometry_case(
            ("w-bodies1-point1-False-foot-None" + suffix).c_str(), jacobian,
            "w", "rider", "bike", kPoint, false, "foot", std::nullopt);
        prepare_geometry_case(
            ("c-bodies2-point2-False-grip-pull2" + suffix).c_str(), jacobian,
            "c", "hand", "bike", std::vector<double>{.2, 0., 1.1}, false,
            "grip", std::vector<double>{-.2, 0., -1.1});
    }
}

// test_attachment_raw_oracle — attachment_raw bitwise on the shared
// solved arena (contacts and a violated joint limit interleave).
void attachment_raw_case(const char *id, const char *eq_name,
                         const char *body_a, const char *body_b,
                         const std::vector<double> &point, bool rotational,
                         const char *kind) {
    const std::string node =
        std::string(kFile) + "test_attachment_raw_oracle[" + id + "]";
    bt::oracle::Node golden(node);
    bt::ModelFixture fixture(
        pair_xml(R"(<weld name="w" body1="rider" body2="bike"/>)"
                 R"(<connect name="c" body1="hand" body2="bike" anchor="0 .1 0"/>)"));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    bool non_equality_rows = false;
    {
        const std::span<const int> efc_types =
            std::views::counted(st->data()->efc_type,
                                st->data()->nefc);
        for (std::size_t i = 0; i < efc_types.size(); ++i)
            non_equality_rows = non_equality_rows ||
                efc_types[i] != mjCNSTR_EQUALITY;
    }
    bt::require(non_equality_rows, "non-equality rows interleave");
    const rw::AttachmentRaw actual = rw::attachment_raw(st->model(), st->data(), st->rider_equalities(), scratch_of(st), 
        equality(m, eq_name), body(m, body_a), body(m, body_b), point,
        kNormal, kind, rotational, .025, std::nullopt);
    assert_raw(golden, actual, "");
    golden.done();
}

void test_attachment_raw_oracle() {
    attachment_raw_case("w-bodies0-point0-True-foot", "w", "rider", "bike",
                        kPoint, true, "foot");
    attachment_raw_case("w-bodies1-point1-False-foot", "w", "rider", "bike",
                        kPoint, false, "foot");
    attachment_raw_case("c-bodies2-point2-False-grip", "c", "hand", "bike",
                        std::vector<double>{.2, 0., 1.1}, false, "grip");
}

// test_attachment_raw_from_geometry_oracle — prepared == eager bitwise,
// the dict round-trip is a binding adapter over the same typed struct,
// and validate_wrench=False defers the spatial checks.
void raw_from_geometry_case(const char *id, const char *eq_name,
                            const char *body_a, const char *body_b,
                            const std::vector<double> &point, bool rotational,
                            const char *kind) {
    const std::string node = std::string(kFile) +
        "test_attachment_raw_from_geometry_oracle[" + id + "]";
    bt::oracle::Node golden(node);
    bt::ModelFixture fixture(
        pair_xml(R"(<weld name="w" body1="rider" body2="bike"/>)"
                 R"(<connect name="c" body1="hand" body2="bike" anchor="0 0 .1"/>)"));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    // Keep every load planar: validate_wrench=True runs the planarity
    // check, and the _drive hand y-load would trip it.
    set_qfrc(m, d, body_dof(m, body(m, "hand")) + 1, 0.);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int eq_id = equality(m, eq_name);
    const int a = body(m, body_a), b = body(m, body_b);
    const rw::AttachmentGeometry geometry = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, a, b, point, kNormal, kind, rotational, .05, std::nullopt);
    assert_raw(golden,
               rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, true), "");
    // vars()-style dicts feed the same typed struct — the binding's
    // attachment_geometry_from is a pure conversion, so the second block
    // replays the identical typed call.
    assert_raw(golden,
               rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, true), "");
    // validate_wrench=False defers every spatial check.
    assert_raw(golden,
               rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, false), "");
    // prepared == eager on an unchanged state, bitwise.
    assert_raw(golden,
               rw::attachment_raw(st->model(), st->data(), st->rider_equalities(), scratch_of(st), eq_id, a, b, point, kNormal, kind,
                                        rotational, .05, std::nullopt),
               "");
    golden.done();
}

void test_attachment_raw_from_geometry_oracle() {
    raw_from_geometry_case("w-bodies0-point0-True-foot", "w", "rider",
                           "bike", kPoint, true, "foot");
    raw_from_geometry_case("w-bodies1-point1-False-foot", "w", "rider",
                           "bike", kPoint, false, "foot");
    raw_from_geometry_case("c-bodies2-point2-False-grip", "c", "hand",
                           "bike", std::vector<double>{.2, 0., 1.1}, false,
                           "grip");
}

void test_attachment_sample_oracle() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(
        pair_xml(R"(<weld name="w" body1="rider" body2="bike"/>)"
                 R"(<connect name="c" body1="hand" body2="bike" anchor="0 0 0"/>)"));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const int hand_dof = body_dof(m, body(m, "hand"));
    set_qfrc(m, d, hand_dof + 1, 0.);  // keep the grip load planar
    set_qfrc(m, d, hand_dof + 2, -7.);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int rider = body(m, "rider"), bike = body(m, "bike");
    const int hand = body(m, "hand");
    const std::vector<double> grip{.2, 0., 1.1};
    struct Case {
        const char *eq;
        int a, b;
        const std::vector<double> *at;
        bool rotational;
        const char *kind;
        std::optional<std::vector<double>> pull;
    };
    const std::array<Case, 4> cases{{
        {"w", rider, bike, &kPoint, true, "foot", std::nullopt},
        {"w", rider, bike, &kPoint, true, "foot",
         std::vector<double>{.1, 0., -.8}},
        {"w", rider, bike, &kPoint, false, "saddle", std::nullopt},
        {"c", hand, bike, &grip, false, "grip",
         std::vector<double>{-.2, 0., -1.1}},
    }};
    for (const Case &c : cases) {
        const std::string msg = std::string(c.eq) + " rot=" +
            (c.rotational ? "True" : "False");
        assert_sample(golden,
                      rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), 
                          equality(m, c.eq), c.a, c.b, *c.at, kNormal,
                          c.kind, c.rotational, .05, c.pull),
                      msg);
    }
    golden.done();
}

void test_decompose_wrench_oracle() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    // 200 replayed cases — the drawn inputs are frozen inputs, the
    // sample fields the comparisons.
    for (int i = 0; i < 200; ++i) {
        const std::string base = "input." + std::to_string(i) + ".";
        const std::vector<double> wrench =
            golden.f64_input(base + "wrench");
        const std::vector<double> normal =
            golden.f64_input(base + "normal");
        const std::string kind =
            decode_f64_text(golden.f64_input(base + "kind"));
        const bool rotational =
            golden.f64_input(base + "rot").at(0) != 0.;
        const std::vector<double> scalars =
            golden.f64_input(base + "scalars");
        bt::require(scalars.size() == 2, "gap/half scalar pair");
        const std::vector<double> pull = golden.f64_input(base + "pull");
        std::optional<std::vector<double>> pull_opt;
        if (!pull.empty()) pull_opt = pull;
        const rw::AttachmentSample actual = rw::decompose_wrench(
            wrench, normal, kind, rotational, scalars[1], scalars[0],
            pull_opt);
        assert_sample(golden, actual, "case " + std::to_string(i));
    }
    // The pull projection gates on the sign, exactly like the oracle.
    const std::vector<double> wrench{3., 0., -2., 0., 0., 0.};
    const std::vector<double> up{0., 0., 1.};
    for (const std::vector<double> &pull :
         std::array{std::vector<double>{1., 0., 0.},
                    std::vector<double>{-1., 0., 0.}})
        assert_sample(golden,
                      rw::decompose_wrench(wrench, up, "grip", false, 0.,
                                           0., pull),
                      "");
    // Error paths — same ValueError, same message.
    const std::vector<double> good{1., 0., -2., 0., .5, 0.};
    const std::vector<double> two{1., 0.};
    const std::vector<double> sideways{0., 1., 0.};
    const std::vector<double> nan_normal{
        std::numeric_limits<double>::quiet_NaN(), 0., 1.};
    const std::vector<double> zero_normal{0., 0., 0.};
    for (const std::vector<double> *bad :
         {&two, &sideways, &nan_normal, &zero_normal})
        expect_error(golden, [&] {
            static_cast<void>(rw::decompose_wrench(good, *bad, "foot",
                                                  true, 0., 0.,
                                                  std::nullopt));
        });
    const std::vector<double> short_pull{1., 0.};
    const std::vector<double> y_pull{0., 1., 0.};
    const std::vector<double> inf_pull{
        std::numeric_limits<double>::infinity(), 0., 1.};
    for (const std::vector<double> *bad : {&short_pull, &y_pull, &inf_pull})
        expect_error(golden, [&] {
            static_cast<void>(rw::decompose_wrench(good, up, "grip", false,
                                                  0., 0., *bad));
        });
    golden.done();
}

void test_attachment_missing_equality_is_no_measurement() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kWeld));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int rider = body(m, "rider"), bike = body(m, "bike");
    // rotational -> the eq_id is only used for row membership: 'no
    // current measurement' is zero force + zero gap.
    const rw::AttachmentSample sample = rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), 
        999, rider, bike, kPoint, kNormal, "foot", true, .05, std::nullopt);
    assert_sample(golden, sample, "");
    bt::require(sample.gap_m == 0. && sample.normal_n == 0.,
                "a missing equality reads no measurement");
    // nonrotational -> eq_type[eq_id] indexes like numpy: IndexError.
    throws_out_of_range([&] {
        static_cast<void>(rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), 
            999, rider, bike, kPoint, kNormal, "foot", false, 0.,
            std::nullopt));
    });
    throws_out_of_range([&] {
        static_cast<void>(rw::attachment_raw(st->model(), st->data(), st->rider_equalities(), scratch_of(st), 
            999, rider, bike, kPoint, kNormal, "foot", false, 0.,
            std::nullopt));
    });
    // Negative eq_id wraps like numpy indexing — never matches rows.
    const rw::AttachmentRaw raw = rw::attachment_raw(st->model(), st->data(), st->rider_equalities(), scratch_of(st), 
        -1, rider, bike, kPoint, kNormal, "foot", false, 0., std::nullopt);
    assert_raw(golden, raw, "");
    bt::require(raw.gap_m == 0., "wrapped id still reads no gap");
    golden.done();
}

void test_attachment_shared_support_error() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kWeld));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int rider = body(m, "rider");
    const int eq_id = equality(m, "w");
    // 'attachment bodies share kinematic support' on every path that
    // joins two supports — prepare, raw, sample in the source's order.
    expect_error(golden, [&] {
        static_cast<void>(rw::prepare_attachment_geometry(st->model(), st->data(), 
            eq_id, rider, rider, kPoint, kNormal, "foot", true, 0.,
            std::nullopt));
    });
    expect_error(golden, [&] {
        static_cast<void>(rw::attachment_raw(st->model(), st->data(), st->rider_equalities(), scratch_of(st), 
            eq_id, rider, rider, kPoint, kNormal, "foot", true, 0.,
            std::nullopt));
    });
    expect_error(golden, [&] {
        static_cast<void>(rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), 
            eq_id, rider, rider, kPoint, kNormal, "foot", true, 0.,
            std::nullopt));
    });
    golden.done();
}

void test_attachment_out_of_plane_error() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kWeld));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    set_qfrc(m, d, body_dof(m, body(m, "rider")) + 1, 40.);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int rider = body(m, "rider"), bike = body(m, "bike");
    const int eq_id = equality(m, "w");
    for (const bool rotational : {false, true})
        expect_error(golden, [&] {
            static_cast<void>(rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), 
                eq_id, rider, bike, kPoint, kNormal, "foot", rotational, 0.,
                std::nullopt));
        });
    const rw::AttachmentGeometry geometry = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, rider, bike, kPoint, kNormal, "foot", true, 0., std::nullopt);
    expect_error(golden, [&] {
        static_cast<void>(
            rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, true));
    });
    // validate_wrench=False returns the raw measurement instead of
    // inventing a planar one.
    assert_raw(golden,
               rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, false),
               "");
    golden.done();
}

void test_attachment_unobservable_and_validation_deferral() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(R"(
<mujoco><option gravity="0 0 0" timestep="0.0005"/>
<worldbody>
  <body name="bike" pos="0 0 1"><freejoint/>
    <inertial pos="0 0 0" mass="1e3" diaginertia="1e3 1e3 1e3"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
  <body name="rider" pos="0 .3 1">
    <joint name="ys" type="slide" axis="0 1 0"/>
    <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
    <geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
</worldbody>
<equality><weld name="w" body1="rider" body2="bike"/></equality>
</mujoco>)");
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    engine::forward(m, d);
    set_qfrc(m, d, jnt_dof(m, joint(m, "ys")), -12.);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int rider = body(m, "rider"), bike = body(m, "bike");
    const int eq_id = equality(m, "w");
    const rw::AttachmentGeometry geometry = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, rider, bike, kPoint, kNormal, "saddle", true, 0.,
        std::nullopt);
    bt::require(!geometry.observable,
                "the y-slide rider keeps a dof but no x/z observability");
    assert_geometry(golden, geometry, "");
    // Eager raw: observable=False is stored, never silently validated.
    assert_raw(golden,
               rw::attachment_raw(st->model(), st->data(), st->rider_equalities(), scratch_of(st), eq_id, rider, bike, kPoint,
                                        kNormal, "saddle", true, 0.,
                                        std::nullopt),
               "");
    // Deferred validation keeps the raw; eager validation raises.
    assert_raw(golden,
               rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, false),
               "");
    expect_error(golden, [&] {
        static_cast<void>(
            rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, true));
    });
    expect_error(golden, [&] {
        static_cast<void>(rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), 
            eq_id, rider, bike, kPoint, kNormal, "saddle", true, 0.,
            std::nullopt));
    });
    golden.done();
}

void test_attachment_newton_third_law_error() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(
        pair_xml(R"(<joint name="j" joint1="ja" joint2="jb"/>)"));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int ja = body(m, "ja_b"), jb = body(m, "jb_b");
    const int eq_id = equality(m, "j");
    const std::vector<double> point{0., 0., .8};
    // A joint equality applies +-tau to two hinge dofs — recovering each
    // side's wrench at the same world point does NOT cancel.
    expect_error(golden, [&] {
        static_cast<void>(rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), 
            eq_id, ja, jb, point, kNormal, "x", false, 0., std::nullopt));
    });
    const rw::AttachmentGeometry geometry = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, ja, jb, point, kNormal, "x", false, 0., std::nullopt);
    expect_error(golden, [&] {
        static_cast<void>(
            rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, true));
    });
    golden.done();
}

void test_attachment_measurement_dicts_own_storage() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kWeld));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    drive(m, d);
    const auto st = mirror(m, d);
    settle(m, d, *st, golden);
    const int rider = body(m, "rider"), bike = body(m, "bike");
    const int eq_id = equality(m, "w");
    rw::AttachmentGeometry first = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, rider, bike, kPoint, kNormal, "foot", true, 0., std::nullopt);
    const rw::AttachmentGeometry second = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, rider, bike, kPoint, kNormal, "foot", true, 0., std::nullopt);
    bt::require(first.rider_jac.values.data() != second.rider_jac.values.data(),
                "each call owns a fresh buffer");
    std::ranges::fill(first.rider_jac.values, -7.);
    assert_geometry(golden, second, "");
    // A prepared snapshot survives stepping: the jacobians still describe
    // the pose they were captured at while raw_from_geometry reads the
    // NEW multipliers of the current solve.
    const rw::AttachmentGeometry snapshot = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, rider, bike, kPoint, kNormal, "foot", true, 0., std::nullopt);
    st->step();
    assert_raw(golden,
               rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), snapshot, true), "");
    // The frozen jacobian is not the post-step one.
    const rw::AttachmentRaw eager = rw::attachment_raw(st->model(), st->data(), st->rider_equalities(), scratch_of(st), 
        eq_id, rider, bike, kPoint, kNormal, "foot", true, 0., std::nullopt);
    bt::require(eager.rider_jac.values != snapshot.rider_jac.values,
                "interval-start jacobians are frozen");
    golden.done();
}

void test_attachment_interval_start_geometry() {
    bt::oracle::Node golden(std::string(kFile) + __func__);
    bt::ModelFixture fixture(pair_xml(kWeld));
    mjModel *m = fixture.model.get();
    mjData *d = fixture.data.get();
    engine::forward(m, d);
    const auto st = mirror(m, d);
    // set_state copies no derived buffers: the owned pair needs its own
    // forward to reach the same post-forward cdof the oracle just read.
    st->forward();
    const int rider = body(m, "rider"), bike = body(m, "bike");
    const int eq_id = equality(m, "w");
    const rw::AttachmentGeometry geometry = rw::prepare_attachment_geometry(st->model(), st->data(), 
        eq_id, rider, bike, kPoint, kNormal, "foot", true, .05,
        std::nullopt);
    assert_geometry(golden, geometry, "");
    const int rider_dof = body_dof(m, rider);
    set_qfrc(m, d, rider_dof, 35.);
    set_qfrc(m, d, rider_dof + 2, 80.);
    set_qfrc(m, d, rider_dof + 4, 7.);
    feed_inputs(m, d, *st);
    for (int i = 0; i < 8; ++i) st->step();
    assert_raw(golden,
               rw::attachment_raw_from_geometry(st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), geometry, true), "");
    assert_sample(golden,
                  rw::attachment_sample(st->model(), st->data(), st->rider_equalities(), lstsq_of(st), scratch_of(st), eq_id, rider, bike, kPoint,
                                              kNormal, "foot", true, .05,
                                              std::nullopt),
                  "");
    golden.done();
}

} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() {
    return bt::run([] {
        test_rider_equality_qfrc_oracle();
        test_rider_equality_rows_grouping();
        test_rider_equality_reaction_oracle();
        test_rider_equality_reaction_missing_rows_zero();
        test_rider_equality_qfrc_empty_arena();
        test_rider_pedal_torque_oracle();
        test_stepper_qfrc_returns_owning_array();
        test_least_squares_oracle();
        test_least_squares_rhs_count();
        test_least_squares_degenerate_dims();
        test_least_squares_rank_deficient();
        test_least_squares_rcond_boundary();
        test_least_squares_rcond_values();
        test_least_squares_inconsistent_target();
        test_least_squares_wide_minimum_norm();
        test_least_squares_nonfinite();
        test_least_squares_shape_errors();
        test_least_squares_incompatible_dimensions_message();
        test_least_squares_output_layout_and_ownership();
        test_linux_and_mac_lstsq_use_numpy_workspace_contract();
        test_least_squares_prior_fp_invalid_flag();
        test_least_squares_fp_invalid_raised_on_failure();
        test_recover_wrench_random_oracle();
        test_recover_wrench_rank_deficient_and_inconsistent();
        test_recover_wrench_rcond_boundary();
        test_recover_wrench_nonfinite();
        test_recover_wrench_degenerate_and_shape_errors();
        test_recover_wrench_owns_its_output();
        test_native_recover_wrench();
        test_relative_planar_jacobian_oracle();
        test_prepare_attachment_geometry_oracle();
        test_attachment_raw_oracle();
        test_attachment_raw_from_geometry_oracle();
        test_attachment_sample_oracle();
        test_decompose_wrench_oracle();
        test_attachment_missing_equality_is_no_measurement();
        test_attachment_shared_support_error();
        test_attachment_out_of_plane_error();
        test_attachment_unobservable_and_validation_deferral();
        test_attachment_newton_third_law_error();
        test_attachment_measurement_dicts_own_storage();
        test_attachment_interval_start_geometry();
    });
}
