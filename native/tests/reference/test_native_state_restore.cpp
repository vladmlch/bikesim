// test_native_state_restore.cpp — C++ port of
// tests/reference/test_native_state_restore.py (T53 batch 3.27).
//
// Two surfaces under test:
//
//   1. Stepper::set_state / set_inputs aliasing, width and finiteness
//      contracts — the binding snapshots every argument span BEFORE
//      mj_resetData, so a live view (even a cross-buffer one, qpos fed as
//      warmstart) restores correctly, and every validation failure leaves
//      the live buffers bitwise untouched.
//   2. Drive-state restore atomicity — DriveWriter::restore decodes and
//      relationally validates a detached snapshot before staging; every
//      rejected mutation leaves state() byte-identical (canonical JSON is
//      the freeze() twin: the encode path is deterministic, so equal
//      snapshots serialize identically).
//
// The two `slow` source nodes replayed a freshly generated 8-step episode
// against a live mujoco mjData. The port replays the FROZEN
// pinned_savage_40_f2 episode (40 mid-ride rows, contacts active —
// richer state coverage than the source's 8 rows) and compares
// Stepper::set_state+forward against engine::forward on a plain mjData
// carrying identical inputs — the exact oracle the source used (MuJoCo
// itself), consumed natively, no Python values. Episode rows are the
// recorded inputs; the forward products are compared native-vs-native,
// bitwise.
//
// Drive-state documents use the state_codec wire Value (the same document
// the binding round-trips): object paths mutate via runtime::setupvalue
// helpers, rejection asserts std::invalid_argument with the source's
// match-needle on the diagnostic path.
#include <mujoco/mujoco.h>

#include "drivetrain_reference_support.hpp"
#include "support/fixture_test.hpp"
#include "support/model_fixture.hpp"
#include "support/temporary_directory.hpp"
#include "tyre_reference_support.hpp"
#include "engine_call.hpp"
#include "golden/episode.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "model_access.hpp"
#include "stepper.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <initializer_list>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using bike::io::Value;
using namespace runtime::setupvalue;

[[nodiscard]] const std::filesystem::path &repo_root() {
    static const std::filesystem::path root = std::filesystem::weakly_canonical(
        bt::oracle::root() / ".." / ".." / "..");
    return root;
}

[[nodiscard]] std::string bench_model_path() {
    return (repo_root() / "tests" / "fixtures" / "benchmark" /
            "model.mjb")
        .string();
}

[[nodiscard]] std::vector<double> linspace(double start, double stop,
                                           std::size_t count) {
    std::vector<double> values(count);
    if (count == 0) return values;
    if (count == 1) {
        values.front() = start;
        return values;
    }
    const double step = (stop - start) / static_cast<double>(count - 1);
    for (std::size_t i = 0; i + 1 < count; ++i)
        values[i] = start + static_cast<double>(i) * step;
    values.back() = stop;
    return values;
}

[[nodiscard]] std::vector<double> copy_of(std::span<const double> values) {
    return {values.begin(), values.end()};
}

void require_bitwise(std::span<const double> actual,
                     std::span<const double> expected,
                     std::string_view context) {
    bt::require(actual.size() == expected.size(),
                std::string(context) + ": widths agree");
    for (std::size_t i = 0; i < actual.size(); ++i)
        bt::require(std::bit_cast<std::uint64_t>(actual[i]) ==
                        std::bit_cast<std::uint64_t>(expected[i]),
                    std::string(context) + ": element " + std::to_string(i) +
                        " is bitwise identical");
}

// The source rig: one hinge joint + one filter-dyn general actuator, so
// nq == nv == na == nu == 1 and every set_state/set_inputs span carries an
// element.
constexpr std::string_view kFilterRig =
    "<mujoco><worldbody><body><joint name=\"a\"/>"
    "<geom size=\".1\" mass=\"1\"/></body></worldbody>"
    "<actuator><general joint=\"a\" dyntype=\"filter\"/></actuator></mujoco>";

[[nodiscard]] std::unique_ptr<Stepper> filter_stepper() {
    return drivetrain_reference::bare(kFilterRig);
}

// _live_buffers/_assert_live_buffers: byte copies of every readable live
// buffer plus the data clock — the atomicity assertion surface.
struct LiveBuffers {
    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> qacc;
    std::vector<double> qfrc_constraint;
    std::vector<double> efc_force;
    std::vector<double> ctrl;
    std::vector<double> qfrc_applied;
    double time = 0.0;
};

[[nodiscard]] LiveBuffers live_buffers(const Stepper &st) {
  return {.qpos = copy_of(st.qpos()),
          .qvel = copy_of(st.qvel()),
          .qacc = copy_of(st.qacc()),
          .qfrc_constraint = copy_of(st.qfrc_constraint()),
          .efc_force = copy_of(st.efc_force()),
          .ctrl = copy_of(st.ctrl()),
          .qfrc_applied = copy_of(st.qfrc_applied()),
          .time = st.time()};
}

void require_live_buffers(const Stepper &st, const LiveBuffers &before) {
    require_bitwise(st.qpos(), before.qpos, "qpos after rejected call");
    require_bitwise(st.qvel(), before.qvel, "qvel after rejected call");
    require_bitwise(st.qacc(), before.qacc, "qacc after rejected call");
    require_bitwise(st.qfrc_constraint(), before.qfrc_constraint,
                    "qfrc_constraint after rejected call");
    require_bitwise(st.efc_force(), before.efc_force,
                    "efc_force after rejected call");
    require_bitwise(st.ctrl(), before.ctrl, "ctrl after rejected call");
    require_bitwise(st.qfrc_applied(), before.qfrc_applied,
                    "qfrc_applied after rejected call");
    bt::require(st.time() == before.time,
                "time after rejected call is unchanged");
}

// ---------------------------------------------------------------------------
// Stepper aliasing + atomicity (lines 17-235 of the source)
// ---------------------------------------------------------------------------

// native_state_restore/set_state_snapshots_own_views_before_reset
void test_set_state_snapshots_own_views_before_reset() {
    const std::string mjb = bench_model_path();
    bike::model::ModelOwner ref_model{engine::load_model(mjb.c_str())};
    std::unique_ptr<mjData, bt::DataDeleter> dref{
        engine::make_data(ref_model.get())};
    const auto qpos = model_access::mutable_buffer(dref->qpos, ref_model->nq);
    const auto qvel = model_access::mutable_buffer(dref->qvel, ref_model->nv);
    qpos[0] += 0.37;
    const auto qvel_values = linspace(-0.2, 0.4, qvel.size());
    std::ranges::copy(qvel_values, qvel.begin());
    Stepper st(mjb);
    st.set_state(model_access::readonly_buffer(dref->qpos, ref_model->nq),
                 model_access::readonly_buffer(dref->qvel, ref_model->nv),
                 model_access::readonly_buffer(dref->act, ref_model->na),
                 model_access::readonly_buffer(dref->qacc_warmstart,
                                               ref_model->nv),
                 0.125);
    st.forward();
    // qacc is another owned buffer resetData clears: aliases are not
    // limited to the destination with the same name.
    const std::vector<double> saved_qpos = copy_of(st.qpos());
    const std::vector<double> saved_qvel = copy_of(st.qvel());
    const std::vector<double> saved_warm = copy_of(st.qacc());
    bt::require(std::ranges::any_of(saved_warm,
                                    [](double v) { return v != 0.0; }),
                "qacc after forward is nonzero — the alias is exercised");
    st.set_state(st.qpos(), st.qvel(),
                 model_access::readonly_buffer(dref->act, ref_model->na),
                 st.qacc(), 0.25);
    require_bitwise(st.qpos(), saved_qpos, "aliased qpos restore");
    require_bitwise(st.qvel(), saved_qvel, "aliased qvel restore");
    bt::require(st.time() == 0.25, "aliased restore advances the clock");
    engine::reset_data(ref_model.get(), dref.get());
    const auto dst_qpos =
        model_access::mutable_buffer(dref->qpos, ref_model->nq);
    const auto dst_qvel =
        model_access::mutable_buffer(dref->qvel, ref_model->nv);
    const auto dst_warm =
        model_access::mutable_buffer(dref->qacc_warmstart, ref_model->nv);
    std::ranges::copy(saved_qpos, dst_qpos.begin());
    std::ranges::copy(saved_qvel, dst_qvel.begin());
    std::ranges::copy(saved_warm, dst_warm.begin());
    dref->time = 0.25;
    engine::forward(ref_model.get(), dref.get());
    st.forward();
    require_bitwise(st.qacc(),
                    model_access::readonly_buffer(dref->qacc, ref_model->nv),
                    "aliased warmstart restore");
    require_bitwise(
        st.qfrc_constraint(),
        model_access::readonly_buffer(dref->qfrc_constraint, ref_model->nv),
        "qfrc_constraint after aliased restore");
}

// A flat C-order row view out of an NpyArray's doubles() payload.
[[nodiscard]] std::span<const double>
episode_row(const std::vector<double> &flat, std::size_t row,
            std::size_t width) {
    bt::require(flat.size() >= (row + 1) * width,
                "episode row inside the decoded extent");
    return std::span<const double>{flat}.subspan(row * width, width);
}

// native_state_restore/forward_on_stored_state_is_bitwise
// The source generated a fresh episode through the research CLI and
// compared st.forward() against a live mj_forward — the MuJoCo oracle. The
// port replays the frozen 40-row pinned_savage_40_f2 episode (the
// post-F2 two-platform fixture) and runs the identical mjData oracle
// in-process: set_state+forward must be bitwise-equal to mj_forward on
// the same buffers.
void test_forward_on_stored_state_is_bitwise() {
    const auto dir = tyre_reference::platform_fixture("episode", "pinned_savage_40_f2");
    const auto episode = bike::golden::load_episode(dir);
    const std::string mjb = (dir / "model.mjb").string();
    bike::model::ModelOwner ref_model{engine::load_model(mjb.c_str())};
    std::unique_ptr<mjData, bt::DataDeleter> dref{
        engine::make_data(ref_model.get())};
    Stepper st(mjb);
    const auto qpos = episode.state_qpos.doubles();
    const auto qvel = episode.state_qvel.doubles();
    const auto act = episode.state_act.doubles();
    const auto warm = episode.state_warmstart.doubles();
    const auto time = episode.state_time.doubles();
    const std::size_t rows = episode.state_qpos.shape.at(0);
    const std::size_t nq = static_cast<std::size_t>(ref_model->nq);
    const std::size_t nv = static_cast<std::size_t>(ref_model->nv);
    const std::size_t na = static_cast<std::size_t>(ref_model->na);
    bt::require(rows > 0 && time.size() == rows,
                "episode carries a nonempty state series");
    for (std::size_t k = 0; k < rows; ++k) {
        const std::span<const double> row_qpos = episode_row(qpos, k, nq);
        const std::span<const double> row_qvel = episode_row(qvel, k, nv);
        const std::span<const double> row_act = episode_row(act, k, na);
        const std::span<const double> row_warm = episode_row(warm, k, nv);
        engine::reset_data(ref_model.get(), dref.get());
        std::ranges::copy(row_qpos,
                          model_access::mutable_buffer(dref->qpos,
                                                       ref_model->nq)
                              .begin());
        std::ranges::copy(row_qvel,
                          model_access::mutable_buffer(dref->qvel,
                                                       ref_model->nv)
                              .begin());
        std::ranges::copy(row_act,
                          model_access::mutable_buffer(dref->act,
                                                       ref_model->na)
                              .begin());
        std::ranges::copy(row_warm,
                          model_access::mutable_buffer(dref->qacc_warmstart,
                                                       ref_model->nv)
                              .begin());
        dref->time = time[k];
        engine::forward(ref_model.get(), dref.get());
        st.set_state(row_qpos, row_qvel, row_act, row_warm, time[k]);
        st.forward();
        const std::string tag = " episode row " + std::to_string(k);
        require_bitwise(
            st.qacc(),
            model_access::readonly_buffer(dref->qacc, ref_model->nv),
            "qacc" + tag);
        require_bitwise(
            st.qfrc_constraint(),
            model_access::readonly_buffer(dref->qfrc_constraint,
                                          ref_model->nv),
            "qfrc_constraint" + tag);
        require_bitwise(
            st.efc_force(),
            model_access::readonly_buffer(dref->efc_force, dref->nefc),
            "efc_force" + tag);
    }
}

// native_state_restore/set_state_size_mismatch_raises_and_keeps_state
void test_set_state_size_mismatch_raises_and_keeps_state() {
    const auto dir = tyre_reference::platform_fixture("episode", "pinned_savage_40_f2");
    const auto episode = bike::golden::load_episode(dir);
    const std::string mjb = (dir / "model.mjb").string();
    bike::model::ModelOwner ref_model{engine::load_model(mjb.c_str())};
    Stepper st(mjb);
    const auto qpos = episode.state_qpos.doubles();
    const auto qvel = episode.state_qvel.doubles();
    const auto act = episode.state_act.doubles();
    const auto warm = episode.state_warmstart.doubles();
    const auto time = episode.state_time.doubles();
    const std::size_t nq = static_cast<std::size_t>(ref_model->nq);
    const std::size_t nv = static_cast<std::size_t>(ref_model->nv);
    const std::size_t na = static_cast<std::size_t>(ref_model->na);
    st.set_state(episode_row(qpos, 0, nq), episode_row(qvel, 0, nv),
                 episode_row(act, 0, na), episode_row(warm, 0, nv), time[0]);
    st.forward();
    const std::vector<double> before = copy_of(st.qacc());
    const std::vector<double> bad_qpos(nq + 1, 0.0);
    drivetrain_reference::error<std::invalid_argument>(
        [&] {
            st.set_state(bad_qpos, episode_row(qvel, 0, nv),
                         episode_row(act, 0, na), episode_row(warm, 0, nv),
                         time[0]);
        });
    // The rejected call must not clobber the previously restored state:
    // every span is width-checked before mj_resetData runs.
    require_bitwise(st.qacc(), before, "qacc after rejected set_state");
}

// native_state_restore/state_rejection_preserves_buffers
void test_state_rejection_preserves_buffers() {
    const std::string mjb = bench_model_path();
    bike::model::ModelOwner ref_model{engine::load_model(mjb.c_str())};
    std::unique_ptr<mjData, bt::DataDeleter> dref{
        engine::make_data(ref_model.get())};
    const auto nv = static_cast<std::size_t>(ref_model->nv);
    const auto nu = static_cast<std::size_t>(ref_model->nu);
    model_access::mutable_buffer(dref->qpos, ref_model->nq)[0] += 0.37;
    const auto qvel_values = linspace(-0.2, 0.4, nv);
    std::ranges::copy(
        qvel_values,
        model_access::mutable_buffer(dref->qvel, ref_model->nv).begin());
    Stepper st(mjb);
    st.set_state(model_access::readonly_buffer(dref->qpos, ref_model->nq),
                 model_access::readonly_buffer(dref->qvel, ref_model->nv),
                 model_access::readonly_buffer(dref->act, ref_model->na),
                 model_access::readonly_buffer(dref->qacc_warmstart,
                                               ref_model->nv),
                 0.125);
    st.forward();
    st.set_inputs(linspace(0.1, 0.3, nu), linspace(-0.1, 0.1, nv));
    const LiveBuffers before = live_buffers(st);
    std::vector<double> bad_qpos(
        model_access::readonly_buffer(dref->qpos, ref_model->nq).begin(),
        model_access::readonly_buffer(dref->qpos, ref_model->nq).end());
    bad_qpos[0] = std::numeric_limits<double>::quiet_NaN();
    drivetrain_reference::error<std::invalid_argument>(
        [&] {
            st.set_state(bad_qpos,
                         model_access::readonly_buffer(dref->qvel,
                                                       ref_model->nv),
                         model_access::readonly_buffer(dref->act,
                                                       ref_model->na),
                         model_access::readonly_buffer(dref->qacc_warmstart,
                                                       ref_model->nv),
                         0.25);
        },
        "set_state.qpos");
    require_live_buffers(st, before);
}

// native_state_restore/set_state_nonfinite_span_rejects_atomically/field-bad_value
void test_set_state_nonfinite_span_rejects_atomically() {
    for (const std::string_view field_name : {"qvel", "act", "warmstart"})
        for (const double bad_value :
             {std::numeric_limits<double>::quiet_NaN(),
              std::numeric_limits<double>::infinity()}) {
            const auto st = filter_stepper();
            const std::vector<double> one{.31}, two{-.2}, three{.45},
                four{.7};
            st->set_state(one, two, three, four, .02);
            st->forward();
            st->set_inputs(std::vector<double>{.6}, std::vector<double>{.4});
            const LiveBuffers before = live_buffers(*st);
            std::vector<double> qpos{.31}, qvel{-.2}, act{.45}, warm{.7};
            if (field_name == "qvel")
                qvel[0] = bad_value;
            else if (field_name == "act")
                act[0] = bad_value;
            else
                warm[0] = bad_value;
            drivetrain_reference::error<std::invalid_argument>(
                [&] { st->set_state(qpos, qvel, act, warm, .03); },
                std::string("set_state.") + std::string(field_name));
            require_live_buffers(*st, before);
        }
}

// native_state_restore/set_state_bad_time_rejects_atomically/bad_time
void test_set_state_bad_time_rejects_atomically() {
    for (const double bad_time :
         {-.5, std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()}) {
        const auto st = filter_stepper();
        st->set_state(std::vector<double>{.31}, std::vector<double>{-.2},
                      std::vector<double>{.45}, std::vector<double>{.7}, .02);
        st->forward();
        const LiveBuffers before = live_buffers(*st);
        drivetrain_reference::error<std::invalid_argument>(
            [&] {
                st->set_state(std::vector<double>{.31},
                              std::vector<double>{-.2},
                              std::vector<double>{.45},
                              std::vector<double>{.7}, bad_time);
            },
            "set_state.time");
        require_live_buffers(*st, before);
    }
}

// native_state_restore/set_inputs_nonfinite_rejects_atomically/field-bad_value
void test_set_inputs_nonfinite_rejects_atomically() {
    for (const std::string_view field_name : {"ctrl", "qfrc_applied"})
        for (const double bad_value :
             {std::numeric_limits<double>::quiet_NaN(),
              -std::numeric_limits<double>::infinity()}) {
            const auto st = filter_stepper();
            st->set_inputs(std::vector<double>{.6}, std::vector<double>{.4});
            const LiveBuffers before = live_buffers(*st);
            std::vector<double> ctrl{.6}, force{.4};
            if (field_name == "ctrl")
                ctrl[0] = bad_value;
            else
                force[0] = bad_value;
            drivetrain_reference::error<std::invalid_argument>(
                [&] { st->set_inputs(ctrl, force); },
                std::string("set_inputs.") + std::string(field_name));
            require_live_buffers(*st, before);
        }
}

// native_state_restore/set_state_accepts_cross_buffer_alias
void test_set_state_accepts_cross_buffer_alias() {
    const std::string mjb = bench_model_path();
    bike::model::ModelOwner ref_model{engine::load_model(mjb.c_str())};
    std::unique_ptr<mjData, bt::DataDeleter> dref{
        engine::make_data(ref_model.get())};
    const auto qpos_target =
        linspace(.11, .22, static_cast<std::size_t>(ref_model->nq));
    const auto qvel_target =
        linspace(-.2, .4, static_cast<std::size_t>(ref_model->nv));
    std::ranges::copy(
        qpos_target,
        model_access::mutable_buffer(dref->qpos, ref_model->nq).begin());
    std::ranges::copy(
        qvel_target,
        model_access::mutable_buffer(dref->qvel, ref_model->nv).begin());
    Stepper st(mjb);
    st.set_state(model_access::readonly_buffer(dref->qpos, ref_model->nq),
                 model_access::readonly_buffer(dref->qvel, ref_model->nv),
                 model_access::readonly_buffer(dref->act, ref_model->na),
                 model_access::readonly_buffer(dref->qacc_warmstart,
                                               ref_model->nv),
                 .125);
    st.forward();
    const std::vector<double> qpos_before = copy_of(st.qpos());
    // The qvel argument is a live view of st's own qpos storage: the
    // binding snapshots every argument before mj_resetData runs.
    const std::span<const double> alias = st.qpos();
    const auto new_qpos =
        linspace(.5, .7, static_cast<std::size_t>(ref_model->nq));
    std::ranges::copy(
        new_qpos,
        model_access::mutable_buffer(dref->qpos, ref_model->nq).begin());
    st.set_state(model_access::readonly_buffer(dref->qpos, ref_model->nq),
                 alias,
                 model_access::readonly_buffer(dref->act, ref_model->na),
                 model_access::readonly_buffer(dref->qacc_warmstart,
                                               ref_model->nv),
                 .5);
    require_bitwise(st.qvel(), qpos_before,
                    "qvel restored through live-qpos alias");
    require_bitwise(st.qpos(),
                    model_access::readonly_buffer(dref->qpos, ref_model->nq),
                    "qpos restored alongside the alias");
    bt::require(st.time() == .5, "aliased restore advances the clock");
}

// native_state_restore/set_state_rejection_with_aliased_buffers_preserves_state
void test_set_state_rejection_with_aliased_buffers_preserves_state() {
    const std::string mjb = bench_model_path();
    bike::model::ModelOwner ref_model{engine::load_model(mjb.c_str())};
    std::unique_ptr<mjData, bt::DataDeleter> dref{
        engine::make_data(ref_model.get())};
    std::ranges::copy(
        linspace(.11, .22, static_cast<std::size_t>(ref_model->nq)),
        model_access::mutable_buffer(dref->qpos, ref_model->nq).begin());
    std::ranges::copy(
        linspace(-.2, .4, static_cast<std::size_t>(ref_model->nv)),
        model_access::mutable_buffer(dref->qvel, ref_model->nv).begin());
    Stepper st(mjb);
    st.set_state(model_access::readonly_buffer(dref->qpos, ref_model->nq),
                 model_access::readonly_buffer(dref->qvel, ref_model->nv),
                 model_access::readonly_buffer(dref->act, ref_model->na),
                 model_access::readonly_buffer(dref->qacc_warmstart,
                                               ref_model->nv),
                 .125);
    st.forward();
    const LiveBuffers before = live_buffers(st);
    // Live-buffer view in the qpos slot; the NaN arrives via qvel.
    std::vector<double> bad_qvel(
        model_access::readonly_buffer(dref->qvel, ref_model->nv).begin(),
        model_access::readonly_buffer(dref->qvel, ref_model->nv).end());
    bad_qvel[3] = std::numeric_limits<double>::quiet_NaN();
    drivetrain_reference::error<std::invalid_argument>(
        [&] {
            st.set_state(st.qpos(), bad_qvel,
                         model_access::readonly_buffer(dref->act,
                                                       ref_model->na),
                         model_access::readonly_buffer(dref->qacc_warmstart,
                                                       ref_model->nv),
                         .75);
        },
        "set_state.qvel");
    require_live_buffers(st, before);
    // A NaN written through the aliased mjData view itself.
    model_access::mutable_buffer(dref->qpos, ref_model->nq)[0] =
        std::numeric_limits<double>::quiet_NaN();
    drivetrain_reference::error<std::invalid_argument>(
        [&] {
            st.set_state(
                model_access::readonly_buffer(dref->qpos, ref_model->nq),
                model_access::readonly_buffer(dref->qvel, ref_model->nv),
                model_access::readonly_buffer(dref->act, ref_model->na),
                model_access::readonly_buffer(dref->qacc_warmstart,
                                              ref_model->nv),
                .75);
        },
        "set_state.qpos");
    require_live_buffers(st, before);
}

// ---------------------------------------------------------------------------
// Drive-state restore atomicity (lines 238-421 of the source)
// ---------------------------------------------------------------------------

// _reject_drive_state: a rejected restore leaves the whole drive snapshot
// untouched — canonical_json is the freeze() twin for the wire document.
void reject_drive_state(const Stepper &owner, const std::function<void(Value &)> &mutate,
                        std::string_view match) {
    const Value before = drivetrain_reference::state(owner);
    Value bad = before;
    mutate(bad);
    drivetrain_reference::error<std::invalid_argument>(
        [&] { drivetrain_reference::restore(owner, bad); }, match);
    bt::require(bike::io::canonical_json(drivetrain_reference::state(owner)) ==
                    bike::io::canonical_json(before),
                "rejected restore leaves drive state untouched (" +
                    std::string(match) + ")");
}

// 'a.b.c' member navigation on the wire document.
[[nodiscard]] Value &at(Value &doc, std::string_view dotted) {
    Value *node = &doc;
    std::string_view rest = dotted;
    while (true) {
        const std::size_t dot = rest.find('.');
        const std::string_view key =
            rest.substr(0, dot == std::string_view::npos ? rest.size() : dot);
        node = &policy_reference::member(*node, key);
        if (dot == std::string_view::npos) return *node;
        rest = rest.substr(dot + 1);
    }
}

[[nodiscard]] const Value &at(const Value &doc, std::string_view dotted) {
    const Value *node = &doc;
    std::string_view rest = dotted;
    while (true) {
        const std::size_t dot = rest.find('.');
        const std::string_view key =
            rest.substr(0, dot == std::string_view::npos ? rest.size() : dot);
        node = &field(*node, key);
        if (dot == std::string_view::npos) return *node;
        rest = rest.substr(dot + 1);
    }
}

// The source's mutate spec language, applied to the leaf value.
void apply_spec(Value &leaf, std::string_view spec,
                const Value &whole_for_over) {
    if (spec == "x1.5") {
        leaf = real(bike::io::as_double(leaf, "leaf") * 1.5);
    } else if (spec == "short") {
        const auto &source = bike::io::as_array(leaf, "leaf");
        bt::require(!source.empty(), "coefficients is nonempty");
        std::vector<double> decoded;
        decoded.reserve(source.size() - 1);
        for (std::size_t i = 0; i + 1 < source.size(); ++i)
            decoded.push_back(bike::io::as_double(source[i], "leaf"));
        leaf = reals(decoded);
    } else if (spec == "i28") {
        leaf = integer(28);
    } else if (spec == "i25") {
        leaf = integer(25);
    } else if (spec == "up") {
        leaf = text("up");
    } else if (spec == "half") {
        leaf = real(.5);
    } else if (spec == "over") {
        const double initial = bike::io::as_double(
            at(whole_for_over, "policies.battery.initial_energy_j"),
            "battery.initial_energy_j");
        leaf = real(initial + 1.);
    } else {
        bt::require(false, "unknown mutation spec");
    }
}

// native_state_restore/drive_restore_relational_rejection_is_atomic/kind-path-spec
void test_drive_restore_relational_rejection_is_atomic() {
    struct Case {
        const char *path;
        const char *spec;
        const char *match;
    };
    constexpr std::array cases{
        // The ideal-hub ratio is always the shifter's current gear.
        Case{.path = "ideal_hub.ratio", .spec = "x1.5", .match = "ratio/gear"},
        // The transmission sprocket tracks shifting.rear_teeth (geometric)
        // or the configured gearing (ideal).
        Case{.path = "ideal_hub.rear_teeth",
             .spec = "i28",
             .match = "rear-teeth"},
        Case{.path = "ideal_hub.coefficients",
             .spec = "short",
             .match = "coefficient width"},
        // The gear-ratio relation reaches the shifter's sprocket even with
        // the shifter disabled (cassette membership applies only when
        // enabled).
        Case{.path = "policies.shifting.rear_teeth",
             .spec = "i25",
             .match = "ratio/gear"},
        // count == 0 requires no shift clock, direction 'none', zero
        // cooldowns.
        Case{.path = "policies.shifting.direction",
             .spec = "up",
             .match = "shift clock/count"},
        Case{.path = "policies.shifting.cooldown_s",
             .spec = "half",
             .match = "shift clock/count"},
        Case{.path = "policies.shifting.cut_remaining_s",
             .spec = "half",
             .match = "shift clock/count"},
        Case{.path = "shift_time_s",
             .spec = "half",
             .match = "shift clock/count"},
        // The store is sized by the config and only ever drains.
        Case{.path = "policies.battery.energy_j",
             .spec = "over",
             .match = "battery energy"},
        Case{.path = "policies.battery.drawn_energy_j",
             .spec = "over",
             .match = "battery energy"},
        Case{.path = "policies.battery.initial_energy_j",
             .spec = "over",
             .match = "battery energy"},
    };
    for (const std::string_view kind :
         {"ideal_mid_drive", "geometric_ideal_mid_drive"})
        for (const auto &[path, spec, match] : cases) {
            const auto owner = drivetrain_reference::pair(kind);
            reject_drive_state(
                *owner,
                [&](Value &bad) {
                    apply_spec(at(bad, path), spec, bad);
                },
                match);
        }
}

// native_state_restore/drive_restore_rejects_coefficient_drift/kind-match
void test_drive_restore_rejects_coefficient_drift() {
    for (const auto &[kind, match] :
         {std::pair{"ideal_mid_drive", "coefficient/ratio"},
          std::pair{"geometric_ideal_mid_drive",
                    "coefficients/prepared Jacobian"}}) {
        const auto owner = drivetrain_reference::pair(kind);
        reject_drive_state(
            *owner,
            [](Value &bad) {
                Value &leaf = at(bad, "ideal_hub.coefficients");
                const auto &source = bike::io::as_array(leaf, "coefficients");
                bt::require(!source.empty(), "coefficients is nonempty");
                std::vector<double> decoded;
                decoded.reserve(source.size());
                for (const auto &element : source)
                    decoded.push_back(
                        bike::io::as_double(element, "coefficients"));
                decoded.front() += 1.;
                leaf = reals(decoded);
            },
            match);
    }
}

// native_state_restore/drive_restore_rejects_geometric_prepared_jacobian_drift
void test_drive_restore_rejects_geometric_prepared_jacobian_drift() {
    const auto owner =
        drivetrain_reference::pair("geometric_ideal_mid_drive");
    reject_drive_state(
        *owner,
        [](Value &bad) {
            Value &leaf = at(bad, "ideal_hub.prepared.jacobian");
            const auto &source = bike::io::as_array(leaf, "jacobian");
            bt::require(!source.empty(), "prepared jacobian is nonempty");
            std::vector<double> decoded;
            decoded.reserve(source.size());
            for (const auto &element : source)
                decoded.push_back(bike::io::as_double(element, "jacobian"));
            decoded.front() += 1.;
            leaf = reals(decoded);
        },
        "coefficients/prepared Jacobian");
}

// native_state_restore/drive_restore_rejects_pending_actuation_without_interval
void test_drive_restore_rejects_pending_actuation_without_interval() {
    const auto owner = drivetrain_reference::pair();
    const Value initial = drivetrain_reference::state(*owner);
    bt::require(bike::io::is_null(field(initial, "pending_actuation")),
                "fresh pair has no pending actuation");
    bt::require(bike::io::is_null(field(initial, "last_time_s")),
                "fresh pair has no interval clock");
    // pending_actuation exists only after a compute that also set the
    // interval clock — both directions of the relation are checked.
    reject_drive_state(
        *owner,
        [](Value &bad) {
            Value::Object pending;
            pending.emplace_back("requested", real(1.));
            pending.emplace_back("omega", real(2.));
            pending.emplace_back("dt", real(.002));
            pending.emplace_back("enabled", Value{true});
            set(bad, "pending_actuation", Value{std::move(pending)});
        },
        "live interval");
    (void)drivetrain_reference::components(*owner, .002, 2.);
    const Value after = drivetrain_reference::state(*owner);
    bt::require(!bike::io::is_null(field(after, "pending_actuation")),
                "drive_components records pending actuation");
    bt::require(!bike::io::is_null(field(after, "last_time_s")),
                "drive_components records the interval clock");
    reject_drive_state(
        *owner, [](Value &bad) { set(bad, "last_time_s", Value{}); },
        "live interval");
}

// native_state_restore/drive_restore_rejects_nonpositive_pending_dt/dt
void test_drive_restore_rejects_nonpositive_pending_dt() {
    for (const double dt : {0., -.002}) {
        const auto owner = drivetrain_reference::pair();
        (void)drivetrain_reference::components(*owner, .002, 2.);
        bt::require(!bike::io::is_null(
                        field(drivetrain_reference::state(*owner),
                              "pending_actuation")),
                    "pending actuation recorded before mutation");
        reject_drive_state(
            *owner,
            [dt](Value &bad) {
                set(policy_reference::member(bad, "pending_actuation"),
                    "dt", real(dt));
            },
            "pending");
    }
}

// native_state_restore/drive_restore_rejects_orphaned_shift_pending_on_ideal_hub
void test_drive_restore_rejects_orphaned_shift_pending_on_ideal_hub() {
    const auto owner = drivetrain_reference::pair();
    reject_drive_state(
        *owner,
        [](Value &bad) {
            set(policy_reference::member(bad, "ideal_hub"), "shift_pending",
                Value{true});
        },
        "shift_pending");
}

// native_state_restore/drive_restore_geometric_shift_pending_accepted
void test_drive_restore_geometric_shift_pending_accepted() {
    const auto owner =
        drivetrain_reference::pair("geometric_ideal_mid_drive");
    Value state = drivetrain_reference::state(*owner);
    bt::require(!bike::io::is_null(at(state, "ideal_hub.prepared")),
                "geometric hub carries a prepared slot after reset");
    set(policy_reference::member(state, "ideal_hub"), "shift_pending",
        Value{true});
    drivetrain_reference::restore(*owner, state);
    bt::require(bike::io::canonical_json(drivetrain_reference::state(*owner)) ==
                    bike::io::canonical_json(state),
                "accepted geometric shift_pending round-trips");
}

// native_state_restore/drive_restore_cassette_membership_when_shifter_enabled
void test_drive_restore_cassette_membership_when_shifter_enabled() {
    auto source = drivetrain_reference::source("ideal_mid_drive");
    source.shifting.enabled = true;
    source.shifting.cadence_smoothing_tau_s = 0.;
    const auto owner = drivetrain_reference::pair(source);
    bt::require(
        bike::io::as_int64(
            at(drivetrain_reference::state(*owner),
               "policies.shifting.shift_count"),
            "shifting.shift_count") == 0,
        "enabled shifter starts at shift_count 0");
    for (const std::string_view key : {"rear_teeth", "from_teeth"})
        reject_drive_state(
            *owner,
            [key](Value &bad) {
                // 25 is outside CASSETTE_12S_TEETH.
                set(at(bad, "policies.shifting"), key, integer(25));
            },
            "cassette");
}

// native_state_restore/drive_restore_rejects_auxiliary_transmission_drift/topology-name-key-value
void test_drive_restore_rejects_auxiliary_transmission_drift() {
    // Clutch/freewheel transmissions are fixed 1:1 one-way clutches with a
    // single unit coefficient — the snapshot cannot retopologize them.
    for (const auto &[topology, name] :
         {std::pair{"clutch", "clutch"}, std::pair{"rotor", "freewheel"}})
        for (const std::size_t which :
             {std::size_t{0}, std::size_t{1}, std::size_t{2},
              std::size_t{3}}) {
            const auto owner = drivetrain_reference::pair(
                "ideal_mid_drive", "crank_effort", topology);
            reject_drive_state(
                *owner,
                [name, which](Value &bad) {
                    Value &member = at(bad, name);
                    switch (which) {
                    case 0:
                        set(member, "ratio", real(1.5));
                        break;
                    case 1:
                        set(member, "rear_teeth", integer(4));
                        break;
                    case 2:
                        static constexpr std::array<double, 1> kSingle{2.};
                        set(member, "coefficients", reals(kSingle));
                        break;
                    default:
                        static constexpr std::array<double, 2> kPair{1., 1.};
                        set(member, "coefficients", reals(kPair));
                        break;
                    }
                },
                "auxiliary");
        }
}

// native_state_restore/drive_policies_set_state_relational_rejections
void test_drive_policies_set_state_relational_rejections() {
    // The standalone policy binding repeats the same relational guards:
    // shift clock/count, cassette membership, battery bounds.
    auto policy = policy_reference::policies(
        policy_reference::config(policy_reference::source(true)));
    const Value before = policy_reference::state(policy);
    struct Case {
        const char *section;
        const char *key;
    };
    constexpr std::array cases{
        Case{.section = "shifting", .key = "direction"},
        Case{.section = "shifting", .key = "cooldown_s"},
        Case{.section = "shifting", .key = "cut_remaining_s"},
        Case{.section = "shifting", .key = "rear_teeth"},
        Case{.section = "battery", .key = "energy_j"},
        Case{.section = "battery", .key = "drawn_energy_j"},
        Case{.section = "battery", .key = "initial_energy_j"},
    };
    const double over = bike::io::as_double(
        field(field(before, "battery"), "initial_energy_j"),
        "battery.initial_energy_j") + 1.;
    for (const auto &[section, key] : cases) {
        Value bad = before;
        Value replacement;
        if (section == std::string_view("shifting") &&
            key == std::string_view("direction"))
            replacement = text("up");
        else if (section == std::string_view("shifting") &&
                 key == std::string_view("cooldown_s"))
            replacement = real(.5);
        else if (section == std::string_view("shifting") &&
                 key == std::string_view("cut_remaining_s"))
            replacement = real(.1);
        else if (section == std::string_view("shifting"))
            replacement = integer(25);
        else
            replacement = real(over);
        set(policy_reference::member(bad, section), key,
            std::move(replacement));
        drivetrain_reference::error<std::invalid_argument>(
            [&] { policy_reference::restore(policy, bad); });
        bt::require(
            bike::io::canonical_json(policy_reference::state(policy)) ==
                bike::io::canonical_json(before),
            std::string("rejected policy restore leaves state untouched (") +
                section + "." + key + ")");
    }
}

void check() {
    test_set_state_snapshots_own_views_before_reset();
    test_forward_on_stored_state_is_bitwise();
    test_set_state_size_mismatch_raises_and_keeps_state();
    test_state_rejection_preserves_buffers();
    test_set_state_nonfinite_span_rejects_atomically();
    test_set_state_bad_time_rejects_atomically();
    test_set_inputs_nonfinite_rejects_atomically();
    test_set_state_accepts_cross_buffer_alias();
    test_set_state_rejection_with_aliased_buffers_preserves_state();
    test_drive_restore_relational_rejection_is_atomic();
    test_drive_restore_rejects_coefficient_drift();
    test_drive_restore_rejects_geometric_prepared_jacobian_drift();
    test_drive_restore_rejects_pending_actuation_without_interval();
    test_drive_restore_rejects_nonpositive_pending_dt();
    test_drive_restore_rejects_orphaned_shift_pending_on_ideal_hub();
    test_drive_restore_geometric_shift_pending_accepted();
    test_drive_restore_cassette_membership_when_shifter_enabled();
    test_drive_restore_rejects_auxiliary_transmission_drift();
    test_drive_policies_set_state_relational_rejections();
}

} // namespace

int main() { return bt::run(check); }
