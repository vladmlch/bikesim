// test_native_engine_errors.cpp — C++ port of
// tests/reference/test_native_engine_errors.py (T53 batch 3.29a).
//
// Source contract: fatal MuJoCo errors must cross the engine boundary
// without exiting the process. The Python suite ran every probe in a
// subprocess because an unhandled vendor fatal would abort the pytest
// process; the production engine boundary (engine::invoke's thread-local
// jump frame) is DESIGNED to recover these fatals in-process, so the
// direct calls below exercise the same recovery paths the Python
// subprocesses observed. Genuinely subprocess-shaped records keep a real
// process boundary through bike::platform::run (the bounded argv spawn
// from task T06): the native_contract_tests allocation probes and the
// scratch configure probe.
//
// Substitutions versus the Python source (each is the same production
// code path; the report's per-record table carries the mapping):
//   * `mujoco.FatalError`      -> `engine::EngineFailure` — the binding's
//     exception translator maps one to the other (binding.cpp).
//   * `_engine_test_forward_failure` / `_engine_test_set_const_failure` /
//     `_engine_test_try_status` -> the identical Stepper::mutate sequences
//     inlined below (binding.cpp:1223-1279) — same tamper, same restore,
//     same poison path.
//   * Python/ctypes timer callbacks -> a foreign-address mjfTime. The
//     Python-free core classifies provenance by address alone, so every
//     non-native owner lands in the same `foreign` bucket the vendored
//     Python trampolines did (stepper_detail::foreign_time_callback_probe
//     stays null without an interpreter).
//   * `_simulate`-image / vendored-stock timers -> `engine::native_time`,
//     the one whitelisted non-null timer owner in the Python-free graph.
//   * the `mujoco.pid` plugin model -> a saved model image whose serialized
//     `nplugin` field is patched to 1: plugin binaries do not ship in the
//     locked .deps tree, and the patched bytes trip the identical
//     `contains_plugins` pre-screen before mj_loadModelBuffer ever parses.
//   * a fake `mujoco` Python package at 3.15.0 -> a scratch
//     BIKE_DEPENDENCY_ROOT whose `mujoco-3.12.0/BIKE_VERSION` marker reads
//     3.15.0; the lock's version-marker gate is the Python-free home of
//     the same pin check.
//
// Dropped as Python-only (see the batch report): the stock trampoline's
// Python-exception propagation (test_stock_time_callback_error_preserves_
// python_exception), the stock-fatal-inside-native-timer nesting
// (test_stock_fatal_handler_nests_inside_native_timer_callback) and the
// stock->native warning non-recursion (test_nested_stock_native_warning_
// does_not_recurse). Those exercise vendored nanobind machinery inside a
// foreign callback — a foreign address is rejected before invocation in
// the Python-free core. Their native halves already live in
// native_contract_tests (nested_engine_frames,
// previous_tls_handler_restored).
#include <mujoco/mujoco.h>

#include "diag.hpp"
#include "drivetrain_reference_support.hpp"
#include "engine_call.hpp"
#include "io/json.hpp"
#include "model_access.hpp"
#include "platform/process.hpp"
#include "stepper.hpp"
#include "support/fixture_test.hpp"
#include "support/model_fixture.hpp"
#include "support/temporary_directory.hpp"

#include "bike_engine_errors_paths.hpp"

#include <bit>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

// The free-body model the source's fatal-injection fixtures use.
constexpr std::string_view kFreeBodyXml = R"XML(<mujoco>
  <worldbody>
    <geom type="plane" size="1 1 .1"/>
    <body pos="0 0 .09">
      <freejoint/>
      <geom type="sphere" size=".1" mass="1"/>
    </body>
  </worldbody>
</mujoco>)XML";

// The degenerate (nq == nv == 0) model the callback fixtures use.
constexpr std::string_view kMinimalXml =
    R"XML(<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>)XML";

// Actuator-history model: delay history forces mjData history buffers,
// which require a positive timestep at mj_makeData time.
constexpr std::string_view kHistoryXml =
    R"XML(<mujoco><worldbody><body><joint name="j" type="slide"/>
    <geom size=".1"/></body></worldbody><actuator>
    <motor joint="j" delay=".05" nsample="5"/></actuator></mujoco>)XML";

void save_mjb(const mjModel &model, const std::filesystem::path &path) {
    mj_saveModel(&model, path.string().c_str(), nullptr, 0);
    bt::require(std::filesystem::exists(path), "mj_saveModel wrote the fixture file");
}

// The source's foreign mjfTime body — never invoked when the engine-layer
// policy works, and the call count proves it.
mjtNum foreign_timer() noexcept;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) test-local probe state
int foreign_timer_calls = 0;
mjtNum foreign_timer() noexcept {
    ++foreign_timer_calls;
    return 0.;
}

void foreign_control(const mjModel *, mjData *) noexcept {}

// Process-global callback slots must return to their prior values so a
// rejection fixture cannot leak into the next contract.
struct CallbackGuard {
    // The snapshot fields stay const: the guard pins the prior callback
    // values for the whole fixture scope.
    const mjfTime previous_time = mjcb_time;       // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members) snapshot member is const by design
    const mjfGeneric previous_control = mjcb_control; // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members) snapshot member is const by design
    CallbackGuard() = default;
    CallbackGuard(const CallbackGuard &) = delete;
    CallbackGuard &operator=(const CallbackGuard &) = delete;
    CallbackGuard(CallbackGuard &&) = delete;
    CallbackGuard &operator=(CallbackGuard &&) = delete;
    ~CallbackGuard() {
        mjcb_time = previous_time;
        mjcb_control = previous_control;
    }
};

// Exact mirror of bike_native._engine_test_forward_failure
// (binding.cpp:1223-1236): corrupt opt.solver inside the mutate boundary,
// run a normal forward, restore the option on every escape path.
void inject_forward_failure(Stepper &stepper) {
    stepper.mutate([&] {
        mjModel *model = stepper.model();
        const int previous_solver = model->opt.solver;
        model->opt.solver = 99;
        try {
            stepper.forward();
        } catch (...) {
            model->opt.solver = previous_solver;
            throw;
        }
        model->opt.solver = previous_solver;
    });
}

// Exact mirror of bike_native._engine_test_set_const_failure
// (binding.cpp:1237-1266): mark body 1 `simple` while its inferred ipos no
// longer matches, so mj_setConst reports its 'simple' diagnostic; restore
// all three mutated fields on every path.
void inject_set_const_failure(Stepper &stepper) {
    stepper.mutate([&] {
        mjModel *model = stepper.model();
        stepper.note_posvel_stale();
        if (model->nbody < 2)
            throw std::invalid_argument("setConst test needs a nonworld body");
        const auto ipos = model_access::mutable_buffer(model->body_ipos, 3 * model->nbody);
        const auto simple = model_access::mutable_buffer(model->body_simple, model->nbody);
        const auto sameframe = model_access::mutable_buffer(model->body_sameframe, model->nbody);
        const auto previous_ipos = ipos[3];
        const auto previous_simple = simple[1];
        const auto previous_sameframe = sameframe[1];
        simple[1] = 1;
        ipos[3] = previous_ipos + .123;
        try {
            engine::set_const(model, stepper.data());
        } catch (...) {
            ipos[3] = previous_ipos;
            simple[1] = previous_simple;
            sameframe[1] = previous_sameframe;
            throw;
        }
        ipos[3] = previous_ipos;
        simple[1] = previous_simple;
        sameframe[1] = previous_sameframe;
    });
}

// Exact mirror of bike_native._engine_test_try_status
// (binding.cpp:1267-1279): the (success, message) pair is produced inside
// the mutate boundary without throwing, while try_step/try_forward's
// internal fatal bookkeeping still poisons the owner.
std::pair<bool, std::string> probe_try_status(Stepper &stepper, bool step) {
    return stepper.mutate([&] {
        stepper.refresh_time_callback_policy();
        mjModel *model = stepper.model();
        const int previous_solver = model->opt.solver;
        model->opt.solver = 99;
        engine::ErrorBuffer error;
        const bool succeeded = step ? stepper.try_step(error)
                                    : stepper.try_forward(error);
        model->opt.solver = previous_solver;
        return std::pair{succeeded, std::string(error.message.data())};
    });
}

[[noreturn]] void fail(std::string_view message) {
    bt::require(false, message);
    std::unreachable();
}

// An EngineFailure escape is the only accepted outcome — the stock
// binding's FatalError surfaces here as the native exception type.
std::string expect_engine_failure(const std::function<void()> &action) {
    try {
        action();
    } catch (const engine::EngineFailure &error) {
        return error.what();
    }
    fail("expected engine::EngineFailure");
}

void expect_invalid_argument(const std::function<void()> &action,
                             std::string_view needle) {
    bool rejected = false;
    std::string message;
    try {
        action();
    } catch (const std::invalid_argument &error) {
        rejected = true;
        message = error.what();
    }
    bt::require(rejected, "expected std::invalid_argument naming the callback contract");
    bt::require(message.find(needle) != std::string::npos,
                "rejection message names the source contract");
}

// The poisoned-owner path throws std::logic_error carrying the canonical
// 'poisoned' sentence (engine::set_poisoned_error via require_healthy).
void expect_poisoned(const std::function<void()> &action) {
    bool rejected = false;
    try {
        action();
    } catch (const std::logic_error &error) {
        rejected =
            std::string_view(error.what()).find("poisoned") != std::string_view::npos;
    }
    bt::require(rejected, "expected the poisoned-owner std::logic_error");
}

// ---- Source: test_stock_binding_raises_fatal_error_and_survives --------
// The stock binding's FatalError is the vendor mapping of this engine
// recovery: a framed mj_forward on an out-of-range solver reports fatal
// status, the frame survives and stays reusable, and the process keeps
// going (the stock Python mapping is bindings-only machinery).
void test_stock_binding_raises_fatal_error_and_survives() {
    bt::ModelFixture invalid(kFreeBodyXml);
    invalid.model->opt.solver = 99;
    bt::ModelFixture const valid(kMinimalXml);
    engine::ErrorBuffer error;
    bt::require(!engine::try_forward(invalid.model.get(), invalid.data.get(), error),
                "the invalid solver reports a failed status, not an exit");
    bt::require(error.kind == engine::ErrorKind::fatal,
                "the invalid solver is a fatal-class engine error");
    bt::require(std::string_view(error.message.data()).find("unknown solver type 99") !=
                    std::string_view::npos,
                "fatal status preserves the engine diagnostic");
    const std::string message = expect_engine_failure(
        [&] { engine::forward(invalid.model.get(), invalid.data.get()); });
    bt::require(message.find("unknown solver type 99") != std::string::npos,
                "EngineFailure carries the engine diagnostic");
    // Process survival + frame reuse: a second framed call on a healthy
    // model succeeds in the same process, twice.
    engine::forward(valid.model.get(), valid.data.get());
    engine::forward(valid.model.get(), valid.data.get());
}

// ---- Source: test_native_fatal_constructor_preserves_process ------------
// Both parametrized import orders collapse to this one native check —
// import order is bindings machinery.
void test_native_fatal_constructor_preserves_process() {
    const bt::TemporaryDirectory scratch("engine-errors-ctor");
    bt::ModelFixture fixture(kFreeBodyXml);
    fixture.model->opt.solver = 99;
    const auto path = scratch.path() / "invalid-solver.mjb";
    save_mjb(*fixture.model, path);
    for (const char *import_order : {"mujoco_first", "native_first"}) {
        (void)import_order; // the two pytest params are bindings import order
        const std::string message = expect_engine_failure(
            [&] { Stepper{path.string()}; });
        bt::require(message.find("unknown solver type 99") != std::string::npos,
                    "the constructor failure preserves the engine diagnostic");
    }
    // The same process still constructs and forwards on a valid model.
    const auto valid = scratch.path() / "valid.mjb";
    bt::ModelFixture const healthy(kMinimalXml);
    save_mjb(*healthy.model, valid);
    Stepper recovered(valid.string());
    recovered.forward();
    recovered.step();
}

// ---- Source: test_valid_solver_still_constructs_and_forwards ------------
void test_valid_solver_still_constructs_and_forwards() {
    const bt::TemporaryDirectory scratch("engine-errors-valid");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "valid.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    stepper.forward();
    stepper.step();
    bt::require(true, "valid forward completed");
}

// ---- Source: test_runtime_fatal_poisons_stepper_until_reset ------------
void test_runtime_fatal_poisons_stepper_until_reset() {
    const bt::TemporaryDirectory scratch("engine-errors-runtime");
    bt::ModelFixture const fixture(kFreeBodyXml);
    const auto path = scratch.path() / "runtime-fatal.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    expect_engine_failure([&] { inject_forward_failure(stepper); });
    expect_poisoned([&] { stepper.forward(); });
    expect_poisoned([&] { stepper.step(); });
    bt::require(stepper.qpos().size() == 7,
                "read-only diagnostics remain accessible while poisoned");
    stepper.reset();
    stepper.forward();
}

// ---- Source: test_set_const_fatal_poisons_owner_and_preserves_process ---
void test_set_const_fatal_poisons_owner_and_preserves_process() {
    const bt::TemporaryDirectory scratch("engine-errors-setconst");
    bt::ModelFixture const fixture(kFreeBodyXml);
    const auto path = scratch.path() / "setconst-fatal.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    const std::string message =
        expect_engine_failure([&] { inject_set_const_failure(stepper); });
    bt::require(message.find("simple") != std::string::npos,
                "setConst failure preserves the engine diagnostic");
    expect_poisoned([&] { stepper.forward(); });
    bt::require(stepper.qpos().size() == 7,
                "read-only diagnostics remain accessible while poisoned");
    stepper.reset();
    stepper.forward();
}

// ---- Source: test_valid_set_state_can_recover_after_rejected_restore ----
void test_valid_set_state_can_recover_after_rejected_restore() {
    const bt::TemporaryDirectory scratch("engine-errors-restore");
    bt::ModelFixture const fixture(kFreeBodyXml);
    const auto path = scratch.path() / "restore-after-fatal.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    const std::vector<double> qpos(stepper.qpos().begin(), stepper.qpos().end());
    const std::vector<double> qvel(stepper.qvel().begin(), stepper.qvel().end());
    const std::vector<double> warmstart(qvel.size(), 0.);
    const std::vector<double> act;
    expect_engine_failure([&] { inject_forward_failure(stepper); });
    // A bad-width restore is rejected BEFORE any buffer mutation, so the
    // owner stays poisoned — the strong-guarantee boundary.
    expect_invalid_argument(
        [&] {
            stepper.set_state(std::span<const double>{qpos}.first(qpos.size() - 1),
                              qvel, act, warmstart, 0.);
        },
        "set_state");
    expect_poisoned([&] { stepper.forward(); });
    stepper.set_state(qpos, qvel, act, warmstart, 0.);
    stepper.forward();
}

// ---- Source: test_configured_owner_recovery_clears_pending_drive_state --
void test_configured_owner_recovery_clears_pending_drive_state() {
    for (const bool use_set_state : {false, true}) {
        auto owner = drivetrain_reference::pair("geometric_ideal_mid_drive");
        static_cast<void>(drivetrain_reference::components(*owner, .002, 2.));
        // DriveSnapshot is several KB — keep each by-value read in its own
        // frame like the drivetrain port does (-Wframe-larger-than).
        const auto snapshot = [&] {
            return std::make_unique<drivetrain::DriveSnapshot>(owner->drive().state());
        };
        {
            const auto before = snapshot();
            bt::require(before->last_time_s.has_value(),
                        "an advancing drive call records its clock");
            bt::require(before->pending_actuation.has_value(),
                        "an advancing drive call stages pending actuation");
        }
        expect_engine_failure([&] { inject_forward_failure(*owner); });
        if (use_set_state) {
            owner->set_state(owner->qpos(), owner->qvel(),
                             model_access::readonly_buffer(owner->data()->act,
                                                           owner->model()->na),
                             model_access::readonly_buffer(owner->data()->qacc_warmstart,
                                                           owner->model()->nv),
                             owner->time());
        } else {
            owner->reset();
        }
        {
            const auto after = snapshot();
            bt::require(!after->last_time_s.has_value(),
                        "recovery clears the recorded drive clock");
            bt::require(!after->pending_actuation.has_value(),
                        "recovery clears staged pending actuation");
        }
        static_cast<void>(drivetrain_reference::components(*owner, .002, 2.));
    }
}

// ---- Source: test_realtime_status_path_reports_fatal_without_throwing ---
void test_realtime_status_path_reports_fatal_without_throwing_inside_marked_region() {
    for (const bool step : {false, true}) {
        const bt::TemporaryDirectory scratch("engine-errors-status");
        bt::ModelFixture const fixture(kFreeBodyXml);
        const auto path = scratch.path() / "realtime-status.mjb";
        save_mjb(*fixture.model, path);
        Stepper stepper(path.string());
        const auto [succeeded, message] = probe_try_status(stepper, step);
        bt::require(!succeeded, "the status path reports failure without throwing");
        bt::require(message.find("unknown solver type 99") != std::string::npos,
                    "the status message preserves the engine diagnostic");
        expect_poisoned([&] { stepper.forward(); });
        stepper.reset();
        stepper.forward();
    }
}

// ---- Source: test_installed_python_callback_is_rejected_before_native_construction
void test_installed_python_callback_is_rejected_before_native_construction() {
    const bt::TemporaryDirectory scratch("engine-errors-cbctor");
    bt::ModelFixture const fixture(kFreeBodyXml);
    const auto path = scratch.path() / "callback.mjb";
    save_mjb(*fixture.model, path);
    {
        const CallbackGuard guard;
        mjcb_control = &foreign_control;
        expect_invalid_argument([&] { Stepper{path.string()}; },
                                "unsupported installed MuJoCo callback: mjcb_control");
    }
    // After the rejection (and with the callback cleared) the same engine
    // remains usable — the stock-binding usability half of the record.
    Stepper stepper(path.string());
    stepper.forward();
}

// ---- Source: test_installed_callback_rejects_runtime_call_without_poison
void test_installed_callback_rejects_runtime_call_without_poison() {
    const bt::TemporaryDirectory scratch("engine-errors-cbrun");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "runtime-callback.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    const double before = stepper.time();
    {
        const CallbackGuard guard;
        mjcb_control = &foreign_control;
        expect_invalid_argument([&] { stepper.forward(); },
                                "unsupported installed MuJoCo callback: mjcb_control");
    }
    bt::require(stepper.time() == before, "rejection leaves engine time untouched");
    stepper.forward();
}

// ---- Source: test_python_time_callback_is_rejected_before_marked_runtime_step
// A Python-managed timer is a foreign address in this graph — the same
// rejection class the vendored trampolines produced.
void test_python_time_callback_is_rejected_before_marked_runtime_step() {
    const bt::TemporaryDirectory scratch("engine-errors-timecb");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "runtime-time-callback.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    const double before = stepper.time();
    {
        const CallbackGuard guard;
        mjcb_time = &foreign_timer;
        expect_invalid_argument([&] { stepper.step(); },
                                "unsupported installed MuJoCo callback: mjcb_time");
    }
    bt::require(stepper.time() == before, "rejection leaves engine time untouched");
    stepper.step();
}

// ---- Source: test_ctypes_time_callback_is_rejected_before_native_construction
void test_ctypes_time_callback_is_rejected_before_native_construction() {
    const bt::TemporaryDirectory scratch("engine-errors-ctorcb");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "ctypes-time-callback.mjb";
    save_mjb(*fixture.model, path);
    foreign_timer_calls = 0;
    {
        const CallbackGuard guard;
        mjcb_time = &foreign_timer;
        expect_invalid_argument([&] { Stepper{path.string()}; },
                                "unsupported installed MuJoCo callback: mjcb_time");
    }
    bt::require(foreign_timer_calls == 0,
                "a foreign timer is rejected by address, never invoked");
}

// ---- Source: test_ctypes_time_callback_is_rejected_before_native_recovery
void test_ctypes_time_callback_is_rejected_before_native_recovery() {
    const bt::TemporaryDirectory scratch("engine-errors-reccb");
    bt::ModelFixture const fixture(
        R"XML(<mujoco><worldbody><body pos=".1 0 0"><geom type="sphere" size=".1"/></body></worldbody></mujoco>)XML");
    const auto path = scratch.path() / "ctypes-recovery-callback.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    stepper.step();
    const double before = stepper.time();
    foreign_timer_calls = 0;
    {
        const CallbackGuard guard;
        mjcb_time = &foreign_timer;
        expect_invalid_argument([&] { stepper.reset(); },
                                "unsupported installed MuJoCo callback: mjcb_time");
        expect_invalid_argument(
            [&] {
                stepper.set_state(stepper.qpos(), stepper.qvel(),
                                  model_access::readonly_buffer(stepper.data()->act,
                                                                stepper.model()->na),
                                  model_access::readonly_buffer(stepper.data()->qacc_warmstart,
                                                                stepper.model()->nv),
                                  before);
            },
            "unsupported installed MuJoCo callback: mjcb_time");
        expect_invalid_argument([&] { inject_set_const_failure(stepper); },
                                "unsupported installed MuJoCo callback: mjcb_time");
    }
    bt::require(foreign_timer_calls == 0,
                "a foreign timer is rejected by address, never invoked");
    bt::require(stepper.time() == before, "rejection leaves engine time untouched");
    stepper.step();
}

// ---- Source: test_simulate_owned_time_callback_is_accepted_for_runtime_steps
// Owner substitution: the _simulate image's Timer has no Python-free
// analog — engine::native_time is the one whitelisted non-null owner the
// provenance policy accepts (stepper_detail::foreign_time_callback_probe
// stays null without an interpreter, so every other address is foreign).
void test_simulate_owned_time_callback_is_accepted_for_runtime_steps() {
    const bt::TemporaryDirectory scratch("engine-errors-simcb");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "simulate-timer-callback.mjb";
    save_mjb(*fixture.model, path);
    {
        const CallbackGuard guard;
        mjcb_time = &engine::native_time;
        Stepper stepper(path.string());
        stepper.step();
        stepper.forward();
        stepper.reset();
        bt::require(stepper.time() == 0., "reset returns the clock to zero");
        stepper.step();
    }
}

// ---- Source: test_unsupported_callback_rejected_before_drive_restore -----
void test_unsupported_callback_rejected_before_drive_restore() {
    auto owner = drivetrain_reference::pair("geometric_ideal_mid_drive");
    const bike::io::Value before = drivetrain_reference::state(*owner);
    bike::io::Value attempted = before;
    // The corrupted snapshot mirrors the Python body's `prepared = None`;
    // the callback gate must fire before the payload is ever decoded.
    runtime::setupvalue::set(policy_reference::member(attempted, "ideal_hub"),
                             "prepared", bike::io::Value{});
    {
        const CallbackGuard guard;
        mjcb_control = &foreign_control;
        expect_invalid_argument(
            [&] {
                owner->mutate(
                    [&] { drivetrain_reference::restore(*owner, attempted); });
            },
            "unsupported installed MuJoCo callback: mjcb_control");
    }
    bt::require(bike::io::canonical_json(drivetrain_reference::state(*owner)) ==
                    bike::io::canonical_json(before),
                "rejection leaves the owner's drive state byte-identical");
}

// ---- Source: test_stock_timer_installed_after_native_construction_is_allowed
// The vendored DefaultTimer's whitelisted owner is engine::native_time in
// this graph: installing it AFTER construction is accepted because the
// realtime path re-classifies provenance on every step.
void test_stock_timer_installed_after_native_construction_is_allowed() {
    const bt::TemporaryDirectory scratch("engine-errors-latecb");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "late-stock-timer.mjb";
    save_mjb(*fixture.model, path);
    Stepper stepper(path.string());
    bt::require(mjcb_time == nullptr, "the fixture starts without a timer callback");
    {
        const CallbackGuard guard;
        mjcb_time = &engine::native_time;
        stepper.step();
    }
}

// ---- Source: test_plugin_model_is_rejected_before_native_engine_loading --
// Deviation (see header): plugin binaries do not ship in the locked .deps
// tree, so the fixture is a saved model image whose serialized `nplugin`
// field is patched to 1 — the identical contains_plugins pre-screen in
// engine::load_model_image rejects before mj_loadModelBuffer parses it.
void test_plugin_model_is_rejected_before_native_engine_loading() {
    // Serialized-model header (engine_call.cpp contains_plugins): five ints,
    // then the MJMODEL_SIZES mjtSize fields; nplugin is index 72.
    constexpr std::size_t kPluginFieldOffset =
        5 * sizeof(int) + 72 * sizeof(mjtSize);
    const bt::TemporaryDirectory scratch("engine-errors-plugin");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "plugin.mjb";
    save_mjb(*fixture.model, path);
    // Valid-input control: the unpatched image constructs fine.
    {
        Stepper control(path.string());
        control.forward();
    }
    std::string image = bt::text(path);
    bt::require(image.size() > kPluginFieldOffset + sizeof(mjtSize),
                "the saved image covers the nplugin field");
    const auto patched = std::bit_cast<std::array<char, sizeof(mjtSize)>>(mjtSize{1});
    std::ranges::copy(patched,
                      std::span{image}.subspan(kPluginFieldOffset, patched.size()).begin());
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out << image;
    }
    expect_invalid_argument([&] { Stepper{path.string()}; },
                            "unsupported MuJoCo plugin model");
}

// ---- Source: test_invalid_history_timestep_rejected_before_make_data -----
void test_invalid_history_timestep_rejected_before_make_data() {
    const bt::TemporaryDirectory scratch("engine-errors-history");
    bt::ModelFixture fixture(kHistoryXml);
    bt::require(fixture.model->nhistory > 0,
                "the actuator-delay fixture owns history buffers");
    fixture.model->opt.timestep = 0;
    const auto path = scratch.path() / "invalid-history-timestep.mjb";
    save_mjb(*fixture.model, path);
    expect_invalid_argument([&] { Stepper{path.string()}; },
                            "history requires positive timestep");
}

// ---- Source: test_bad_model_file_raises_without_exiting ------------------
void test_bad_model_file_raises_without_exiting() {
    const bt::TemporaryDirectory scratch("engine-errors-bad");
    for (const int payload : {0, 1, 2}) {
        const auto path = scratch.path() / (std::string("bad") + std::to_string(payload) + ".mjb");
        if (payload == 1) {
            std::ofstream(path, std::ios::binary | std::ios::trunc);
        } else if (payload == 2) {
            std::ofstream(path, std::ios::binary | std::ios::trunc) << "not an MJB";
        } // payload 0 leaves the file absent, like the Python None case
        bool fatal = false;
        bool runtime = false;
        try {
            Stepper{path.string()};
        } catch (const engine::EngineFailure &) {
            fatal = true;
        } catch (const std::runtime_error &) {
            runtime = true;
        }
        bt::require(!fatal, "an ordinary bad MJB must not become a fatal engine error");
        bt::require(runtime, "a bad MJB is a plain load rejection, not an exit");
    }
    // Process survival: a valid owner still constructs after all three.
    bt::ModelFixture const fixture(kMinimalXml);
    const auto valid = scratch.path() / "valid.mjb";
    save_mjb(*fixture.model, valid);
    Stepper recovered(valid.string());
    recovered.forward();
}

// ---- Source: test_actual_engine_allocation_error_is_caught_in_c_contract --
// The Python suite spawns the selected build's native_contract_tests binary
// for the three allocation probes; the port does the same through the
// bounded platform::run argv boundary — real subprocess isolation for a
// fault injection that must not share this test's heap.
void test_actual_engine_allocation_error_is_caught_in_c_contract() {
    const bt::TemporaryDirectory scratch("engine-errors-alloc");
    bt::ModelFixture const fixture(kMinimalXml);
    const auto path = scratch.path() / "valid-for-allocation.mjb";
    save_mjb(*fixture.model, path);
    bike::platform::ProcessOptions options;
    options.timeout_s = 30.;
    for (const char *operation :
         {"--load-allocation-failure", "--make-data-allocation-failure",
          "--late-data-allocation-failure"}) {
        const auto result = bike::platform::run(
            {BIKE_CONTRACT_TESTS_PATH, operation, path.string()},
            BIKE_ENGINE_ERRORS_REPO_ROOT, options);
        bt::require(result.exit_code == 0,
                    (std::string("allocation probe exits cleanly: ") += operation).c_str());
        bt::require(result.out.find("fatal allocation") != std::string::npos,
                    (std::string("allocation probe reports the C-contract verdict: ") +=
                     operation)
                        .c_str());
    }
}

// ---- Source: test_configure_rejects_unsupported_installed_mujoco_version --
// Deviation (see header): the Python body fakes a `mujoco` package at
// 3.15.0; the Python-free graph's equivalent gate is the lock's
// BIKE_VERSION marker check inside bike_dependencies_resolve — the same
// configure-time rejection, before any license/archive/hash gate.
void test_configure_rejects_unsupported_installed_mujoco_version() {
    const bt::TemporaryDirectory scratch("engine-errors-deps");
    // The resolve loop verifies dependencies alphabetically (CMake's JSON
    // member order), so every dep ahead of mujoco must pass its real checks:
    // link the genuine acquired trees and replace only the mujoco root with
    // a directory whose BIKE_VERSION marker declares the wrong version. The
    // marker gate fires before that dep's own license/archive/hash checks.
    for (const auto &entry :
         std::filesystem::directory_iterator(BIKE_ENGINE_ERRORS_DEPENDENCY_ROOT)) {
        if (entry.path().filename() == "mujoco-3.12.0") continue;
        std::filesystem::create_directory_symlink(
            entry.path(), scratch.path() / entry.path().filename());
    }
    const auto dep = scratch.path() / "mujoco-3.12.0";
    std::filesystem::create_directories(dep);
    {
        std::ofstream marker(dep / "BIKE_VERSION", std::ios::binary | std::ios::trunc);
        marker << "3.15.0\n";
    }
    bike::platform::ProcessOptions options;
    options.timeout_s = 120.;
    const auto result = bike::platform::run(
        {BIKE_ENGINE_ERRORS_CMAKE_COMMAND,
         "-S", BIKE_ENGINE_ERRORS_REPO_ROOT,
         "-B", (scratch.path() / "build").string(),
         "-DBIKE_DEPENDENCY_ROOT=" + scratch.path().string(),
         "-DBIKE_DEPENDENCY_LOCK=" BIKE_ENGINE_ERRORS_DEPENDENCY_LOCK, // NOLINT(bugprone-suspicious-missing-comma) adjacent literals intentionally join the fragments
         "-DBIKE_BUILD_GUI=OFF"},
        BIKE_ENGINE_ERRORS_REPO_ROOT, options);
    const std::string combined = result.out + result.err;
    bt::require(result.exit_code != 0,
                "configure refuses a wrong-version MuJoCo dependency root");
    bt::require(combined.find("mujoco") != std::string::npos,
                "the configure failure names the mujoco dependency");
    bt::require(combined.find("requires version '3.12.0'") != std::string::npos,
                "the configure failure carries the locked 3.12.0 pin");
    bt::require(combined.find("'3.15.0'") != std::string::npos,
                "the configure failure names the provided wrong version");
}

} // namespace

// The 21 test bodies inline into this one lambda; the aggregate frame
// exceeds the 8 KiB policy under clang even though each callee stays
// within budget, so the suppression wraps the dispatcher only.
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_stock_binding_raises_fatal_error_and_survives();
        test_native_fatal_constructor_preserves_process();
        test_valid_solver_still_constructs_and_forwards();
        test_runtime_fatal_poisons_stepper_until_reset();
        test_set_const_fatal_poisons_owner_and_preserves_process();
        test_valid_set_state_can_recover_after_rejected_restore();
        test_configured_owner_recovery_clears_pending_drive_state();
        test_realtime_status_path_reports_fatal_without_throwing_inside_marked_region();
        test_installed_python_callback_is_rejected_before_native_construction();
        test_installed_callback_rejects_runtime_call_without_poison();
        test_python_time_callback_is_rejected_before_marked_runtime_step();
        test_ctypes_time_callback_is_rejected_before_native_construction();
        test_ctypes_time_callback_is_rejected_before_native_recovery();
        test_simulate_owned_time_callback_is_accepted_for_runtime_steps();
        test_unsupported_callback_rejected_before_drive_restore();
        test_stock_timer_installed_after_native_construction_is_allowed();
        test_plugin_model_is_rejected_before_native_engine_loading();
        test_invalid_history_timestep_rejected_before_make_data();
        test_bad_model_file_raises_without_exiting();
        test_actual_engine_allocation_error_is_caught_in_c_contract();
        test_configure_rejects_unsupported_installed_mujoco_version();
    });
}
NATIVE_DIAG_POP
