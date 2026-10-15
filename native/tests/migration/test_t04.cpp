// test_t04.cpp — T04 regression for the pure engine boundary: resolved
// runtime identity, native/foreign callback provenance, nested-handler
// restoration, thread-independent frames, and fatal committed-prefix
// behaviour. Every case runs against the same bike_engine /
// bike_native_engine TU the standalone and binding builds share.
//
// Callback and fatal controls mutate process-global engine slots (mjcb_*)
// and deliberately trip MuJoCo fatals. Each such case runs in a forked
// child so a regression aborts only the child and reaches the parent as a
// failed check — the same isolation the Python-side fatal tests get from
// pytest's per-test process model.
#include "support/migration_test.hpp"
#include "engine_call.hpp"
#include "engine_identity.hpp"

#include <mujoco/mujoco.h>

#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <bit>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <ranges>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

    // Forked-control runner (POSIX — the engine layer targets macOS). The
    // child executes `body` through bt::run, so the zero-checks fail-closed
    // rule and the PASS/FAIL line apply inside the child too; _Exit — not
    // exit — keeps atexit handlers off the parent's copied fd/global state.
    bool in_child(const std::function<void()> &body) {
        // Drain the parent's buffers before the copy; the flush result is
        // advisory here — a failed flush only repeats output, never hides it.
        static_cast<void>(std::fflush(nullptr));
        const pid_t pid = ::fork();
        bt::require(pid >= 0, "fork failed for a fatal/callback control");
        if (pid == 0) std::_Exit(bt::run(body));
        int status = 0;
        pid_t waited = ::waitpid(pid, &status, 0);
        while (waited < 0 && errno == EINTR)
            waited = ::waitpid(pid, &status, 0);
        bt::require(waited == pid, "waitpid failed for a control child");
        return WIFEXITED(status) != 0 && WEXITSTATUS(status) == 0;
    }

    // Minimal single-field extractor for the authored engine/identity.json
    // descriptor — flat unescaped string values only, so a full JSON
    // dependency would be pure weight. Fails closed on a missing or
    // escaped field. `key` is a literal (const char*) so the parameters
    // cannot be swapped silently.
    std::string json_string(std::string_view document, const char *key) {
        const std::string needle = "\"" + std::string(key) + "\"";
        const std::size_t at = document.find(needle);
        bt::require(at != std::string_view::npos,
                    "identity fixture field missing: " + std::string(key));
        const std::size_t colon = document.find(':', at + needle.size());
        bt::require(colon != std::string_view::npos,
                    "identity fixture malformed after key: " +
                        std::string(key));
        const std::size_t open = document.find('"', colon + 1);
        const std::size_t close =
            open == std::string_view::npos
                ? open
                : document.find('"', open + 1);
        bt::require(open != std::string_view::npos &&
                        close != std::string_view::npos,
                    "identity fixture value malformed: " + std::string(key));
        const std::string value{document.substr(open + 1, close - open - 1)};
        bt::require(value.find('\\') == std::string::npos,
                    "identity fixture value unexpectedly escaped: " +
                        std::string(key));
        return value;
    }

    // Reachability counter shared between the mjfTime-shaped callback (a
    // free function, so no context pointer) and the test body — necessarily
    // a mutable global; atomic keeps it data-race-free across child
    // processes' inherited threads.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    std::atomic<long> foreign_timer_calls{0};

    // A foreign timer: a valid plain-C callback the engine will call at
    // timer sites inside marked work. Not the documented native timer, not
    // a vendor image — provenance must classify it foreign. Reachability
    // (the counter) is separate from acceptance (the policy result).
    mjtNum foreign_timer() noexcept {
        foreign_timer_calls.fetch_add(1, std::memory_order_relaxed);
        return 0.001;
    }
    void foreign_control(const mjModel *, mjData *) noexcept {}

    struct TimerSlotGuard {
        TimerSlotGuard() = default;
        TimerSlotGuard(const TimerSlotGuard &) = delete;
        TimerSlotGuard &operator=(const TimerSlotGuard &) = delete;
        TimerSlotGuard(TimerSlotGuard &&) = delete;
        TimerSlotGuard &operator=(TimerSlotGuard &&) = delete;
        ~TimerSlotGuard() { mjcb_time = saved; }

      private:
        mjfTime saved = mjcb_time;
    };
    struct ControlSlotGuard {
        ControlSlotGuard() = default;
        ControlSlotGuard(const ControlSlotGuard &) = delete;
        ControlSlotGuard &operator=(const ControlSlotGuard &) = delete;
        ControlSlotGuard(ControlSlotGuard &&) = delete;
        ControlSlotGuard &operator=(ControlSlotGuard &&) = delete;
        ~ControlSlotGuard() { mjcb_control = saved; }

      private:
        decltype(mjcb_control) saved = mjcb_control;
    };

    using ModelPtr = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
    using DataPtr = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

    ModelPtr load_model_fixture(const std::filesystem::path &path) {
        return {engine::load_model(path.string().c_str()), &mj_deleteModel};
    }

    // Bitwise binary64 equality — object representations, so signed zero
    // and NaN payloads are significant (the same rule bt::golden_f64 uses).
    bool bits_equal(const mjtNum *actual, const std::vector<double> &expected,
                    std::size_t count) {
        return std::ranges::equal(
            std::views::counted(actual, static_cast<std::ptrdiff_t>(count)),
            expected, [](mjtNum a, double b) {
                return std::bit_cast<std::uint64_t>(a) ==
                       std::bit_cast<std::uint64_t>(b);
            });
    }
    std::vector<double> copy_doubles(const mjtNum *source, std::size_t count) {
        std::vector<double> copy(count);
        std::ranges::copy(
            std::views::counted(source, static_cast<std::ptrdiff_t>(count)),
            copy.begin());
        return copy;
    }

    // ---- nested-op context: exercises an inner framed fatal, an inner
    // framed success, then a RAW MuJoCo fatal that must land on the outer
    // frame — the same nesting the stock/native handler-restoration Python
    // tests cover.
    struct NestedContext {
        const mjModel *good_model = nullptr;
        const mjModel *bad_model = nullptr;
        mjData *data = nullptr;
        bool inner_fatal_contained = false;
        bool inner_ok_after_fatal = false;
        bool reached_after_raw_fatal = false;
    };
    void nested_op(void *raw) noexcept {
        auto *context = static_cast<NestedContext *>(raw);
        engine::ErrorBuffer inner{};
        context->inner_fatal_contained =
            !engine::try_step(context->bad_model, context->data, inner) &&
            inner.kind == engine::ErrorKind::fatal;
        // A normal inner framed call after the contained fatal proves the
        // inner frame restored the OUTER frame's handler, not MuJoCo's
        // default.
        engine::ErrorBuffer inner2{};
        context->inner_ok_after_fatal = engine::try_forward_skip(
            context->good_model, context->data, inner2);
        // Raw engine call without a frame: the fatal must longjmp to the
        // outer frame, never return here.
        mj_step(context->bad_model, context->data);
        context->reached_after_raw_fatal = true;
    }

    // ---- Child bodies live at file scope: the legacy build enforces
    // -Wframe-larger-than=8192 and a single ErrorBuffer is already ~2 KiB.
    void callback_provenance_body(const std::filesystem::path &model_path) {
        const TimerSlotGuard timer_guard;
        const ControlSlotGuard control_guard;

        // no callback
        mjcb_time = nullptr;
        bt::require(engine::time_callback_provenance() ==
                        engine::TimeCallbackProvenance::none,
                    "provenance: nullptr must classify as none");
        engine::ErrorBuffer error{};
        bt::require(engine::try_native_time_policy(error) &&
                        error.kind == engine::ErrorKind::none,
                    "policy: no timer must be accepted");

        // documented native timer
        mjcb_time = &engine::native_time;
        bt::require(engine::time_callback_provenance() ==
                        engine::TimeCallbackProvenance::native,
                    "provenance: &engine::native_time must classify as "
                    "native");
        bt::require(engine::try_native_time_policy(error) &&
                        error.kind == engine::ErrorKind::none,
                    "policy: the native timer must be accepted");
        const mjtNum t1 = engine::native_time();
        const mjtNum t2 = engine::native_time();
        bt::require(t1 >= 0 && t2 >= t1,
                    "native_time must be monotone nonnegative "
                    "steady-clock seconds");

        // foreign timer: classified foreign, rejected verbatim
        mjcb_time = &foreign_timer;
        bt::require(engine::time_callback_provenance() ==
                        engine::TimeCallbackProvenance::foreign,
                    "provenance: a plain-C timer must classify as "
                    "foreign");
        error = {};
        bt::require(
            !engine::try_native_time_policy(error) &&
                error.kind == engine::ErrorKind::unsupported_callback &&
                std::string_view(error.message.data()) ==
                    "unsupported installed MuJoCo callback: "
                    "mjcb_time",
            "policy: a foreign timer must be rejected with the "
            "canonical message");
        bt::invalid([] { engine::require_native_time_policy(); });

        // invoke's own callback scan is callback-agnostic for
        // the other slots: a foreign mjcb_control is rejected
        // verbatim even inside load_model's frame.
        mjcb_control = &foreign_control;
        bool control_rejected = false;
        try {
            static_cast<void>(load_model_fixture(model_path));
        } catch (const std::invalid_argument &rejection) {
            control_rejected =
                std::string_view(rejection.what()) ==
                "unsupported installed MuJoCo callback: "
                "mjcb_control";
        }
        bt::require(control_rejected,
                    "invoke must reject a foreign mjcb_control "
                    "with the canonical message");
        mjcb_control = nullptr;

        // A foreign timer inside a marked region is still a
        // plain C call: reachability is harmless, acceptance is
        // the policy's job — the counter proves the marked step
        // really did read mjcb_time.
        const ModelPtr model = load_model_fixture(model_path);
        bt::require(model != nullptr, "minimal.mjb must load");
        const DataPtr data(engine::make_data(model.get()),
                           &mj_deleteData);
        foreign_timer_calls.store(0, std::memory_order_relaxed);
        error = {};
        bt::require(engine::try_step(model.get(), data.get(), error),
                    "step with a foreign C timer installed must "
                    "succeed");
        bt::require(
            foreign_timer_calls.load(std::memory_order_relaxed) > 0,
            "the marked step must have reached mjcb_time");
    }

    void committed_prefix_body(const std::filesystem::path &model_path) {
        const ModelPtr model = load_model_fixture(model_path);
        const DataPtr data(engine::make_data(model.get()),
                           &mj_deleteData);
        bt::require(model != nullptr && data != nullptr,
                    "minimal.mjb must load");

        // ErrorBuffer is ~2 KiB — heap keeps this body under the legacy
        // build's -Wframe-larger-than=8192.
        const auto error = std::make_unique<engine::ErrorBuffer>();
        bt::require(engine::try_step(model.get(), data.get(), *error),
                    "baseline step must succeed");
        const mjtNum committed_time = data->time;
        const int committed_steps = data->timer[mjTIMER_STEP].number;
        const std::vector<double> committed_qpos =
            copy_doubles(data->qpos, static_cast<std::size_t>(model->nq));
        const std::vector<double> committed_qvel =
            copy_doubles(data->qvel, static_cast<std::size_t>(model->nv));
        bt::require(committed_time > 0 && committed_steps == 1,
                    "the baseline step must have committed");

        // Out-of-range opt.integrator reaches mj_step's verbatim
        // mju_error("invalid integrator") — a deterministic
        // fatal AFTER the forward committed its products.
        model->opt.integrator = -1;
        *error = {};
        bt::require(
            !engine::try_step(model.get(), data.get(), *error) &&
                error->kind == engine::ErrorKind::fatal,
            "invalid integrator must surface as a contained "
            "fatal, not an abort");
        bt::require(std::string_view(error->message.data())
                            .find("invalid integrator") !=
                        std::string_view::npos,
                    "fatal message must carry 'invalid integrator' — "
                    "got '" +
                        std::string(error->message.data()) + "'");

        // Committed prefix: the failed step advances nothing —
        // prior commits stay readable and bit-identical.
        bt::require(
            data->time == committed_time &&
                data->timer[mjTIMER_STEP].number == committed_steps &&
                bits_equal(data->qpos, committed_qpos,
                           static_cast<std::size_t>(model->nq)) &&
                bits_equal(data->qvel, committed_qvel,
                           static_cast<std::size_t>(model->nv)),
            "the failed step must not advance time, integrator "
            "counters, qpos or qvel past the committed prefix");

        // The engine layer itself is stateless — a fresh framed
        // call works immediately. The poison-until-reset
        // contract lives at the Stepper adapter, which formats
        // it through the exported set_poisoned_error.
        model->opt.integrator = mjINT_EULER;
        *error = {};
        bt::require(engine::try_step(model.get(), data.get(), *error),
                    "a fresh framed call must work after a contained "
                    "fatal");
        const auto poisoned = std::make_unique<engine::ErrorBuffer>();
        engine::set_poisoned_error(*poisoned);
        bt::require(
            poisoned->kind == engine::ErrorKind::poisoned &&
                std::string_view(poisoned->message.data()) ==
                    "Stepper is poisoned after a fatal MuJoCo "
                    "error; call reset",
            "poisoned error must keep its canonical kind and "
            "message");
    }

    void nested_and_threads_body(
        const std::filesystem::path &model_path) {
        const ModelPtr good = load_model_fixture(model_path);
        const ModelPtr bad = load_model_fixture(model_path);
        const DataPtr data(engine::make_data(good.get()),
                           &mj_deleteData);
        bt::require(good != nullptr && bad != nullptr && data != nullptr,
                    "minimal.mjb must load");
        bad->opt.integrator = -1;

        NestedContext context{.good_model = good.get(),
                              .bad_model = bad.get(),
                              .data = data.get()};
        const auto outer = std::make_unique<engine::ErrorBuffer>();
        const bool outer_ok =
            engine::invoke(&nested_op, &context, *outer);
        bt::require(!outer_ok && outer->kind == engine::ErrorKind::fatal,
                    "a raw MuJoCo fatal inside the operation "
                    "must land on the outer frame");
        bt::require(context.inner_fatal_contained &&
                        context.inner_ok_after_fatal &&
                        !context.reached_after_raw_fatal,
                    "nested frames must contain their own "
                    "fatal, then keep working after it");

        // Thread-independent contexts: each worker holds its
        // own frames/error state; one worker's fatal must not
        // leak into the other worker or the main thread.
        std::atomic<int> finished{0};
        std::atomic<int> failures{0};
        const auto worker = [&] {
            const ModelPtr worker_good = load_model_fixture(model_path);
            const ModelPtr worker_bad = load_model_fixture(model_path);
            worker_bad->opt.integrator = -1;
            const DataPtr worker_data(
                engine::make_data(worker_good.get()), &mj_deleteData);
            // ~2 KiB each — heap keeps the lambda under
            // -Wframe-larger-than=8192.
            const auto error = std::make_unique<engine::ErrorBuffer>();
            const bool before =
                engine::try_step(worker_good.get(), worker_data.get(),
                                 *error);
            const auto fatal = std::make_unique<engine::ErrorBuffer>();
            const bool contained =
                !engine::try_step(worker_bad.get(), worker_data.get(),
                                  *fatal) &&
                fatal->kind == engine::ErrorKind::fatal;
            const auto after = std::make_unique<engine::ErrorBuffer>();
            const bool recovered =
                engine::try_step(worker_good.get(), worker_data.get(),
                                 *after);
            if (!(before && contained && recovered))
                failures.fetch_add(1, std::memory_order_relaxed);
            finished.fetch_add(1, std::memory_order_relaxed);
        };
        std::thread first(worker);
        std::thread second(worker);
        first.join();
        second.join();
        bt::require(
            finished.load(std::memory_order_relaxed) == 2 &&
                failures.load(std::memory_order_relaxed) == 0,
            "each worker thread must contain its own fatal "
            "independently");
        const auto error = std::make_unique<engine::ErrorBuffer>();
        bt::require(
            engine::try_forward_skip(good.get(), data.get(), *error),
            "the main thread must be unaffected by worker "
            "fatals");
    }

} // namespace

int main() {
    // cppcheck's throwInEntryPoint cannot see that bt::run catches every
    // test escape; the outer try covers std::function allocation failure.
    try {
        return bt::run([] {
            const std::filesystem::path model_path =
                bt::fixture("models/minimal.mjb");

            // ---- identity: resolved triple vs the authored descriptor ----
            const std::string descriptor =
                bt::text(bt::fixture("engine/identity.json"));
            const engine::Identity required{
                .mujoco_version =
                    json_string(descriptor, "mujoco_version"),
                .library_sha256 =
                    json_string(descriptor, "mujoco_library_sha256"),
                .numeric_provider =
                    json_string(descriptor, "numeric_provider"),
            };
            const engine::Identity resolved = engine::identity();
            bt::require(resolved.mujoco_version == required.mujoco_version,
                        "identity: mujoco_version mismatch — resolved '" +
                            resolved.mujoco_version + "'");
            bt::require(resolved.library_sha256 ==
                            required.library_sha256,
                        "identity: loaded-library sha256 mismatch — resolved '" +
                            resolved.library_sha256 + "'");
            bt::require(resolved.numeric_provider ==
                            required.numeric_provider,
                        "identity: numeric_provider mismatch — resolved '" +
                            resolved.numeric_provider + "'");
            engine::verify_identity(required);
            // Field-by-field mismatch reporting: each tampered field throws.
            bt::invalid([&] {
                engine::Identity tampered = required;
                tampered.mujoco_version = "0.0.0";
                engine::verify_identity(tampered);
            });
            bt::invalid([&] {
                engine::Identity tampered = required;
                tampered.library_sha256 = std::string(64, '0');
                engine::verify_identity(tampered);
            });
            bt::invalid([&] {
                engine::Identity tampered = required;
                tampered.numeric_provider = "mkl";
                engine::verify_identity(tampered);
            });

            // ---- non-MJB payload: load_model returns nullptr; MuJoCo's
            // header warnings forward to the pre-frame destination. ----
            {
                const ModelPtr rejected =
                    load_model_fixture(bt::fixture("harness/utf8.txt"));
                bt::require(rejected == nullptr,
                            "non-MJB payload must return nullptr");
            }

            // ---- bitwise provider determinism at the boundary: identical
            // forwards produce bit-identical products on the recorded
            // provider. ----
            {
                const ModelPtr model = load_model_fixture(model_path);
                bt::require(model != nullptr, "minimal.mjb must load");
                const DataPtr data(engine::make_data(model.get()),
                                   &mj_deleteData);
                bt::require(data != nullptr,
                            "make_data must produce a valid mjData");
                engine::forward(model.get(), data.get());
                const std::vector<double> qfrc_first =
                    copy_doubles(data->qfrc_smooth,
                                 static_cast<std::size_t>(model->nv));
                engine::reset_data(model.get(), data.get());
                engine::forward(model.get(), data.get());
                bt::require(
                    bits_equal(data->qfrc_smooth, qfrc_first,
                               static_cast<std::size_t>(model->nv)),
                    "same-provider forward must be bit-identical across "
                    "identical inputs");
            }

            // ---- callback provenance (child: installs process-global
            // callback slots) ----
            bt::require(
                in_child([&] { callback_provenance_body(model_path); }),
                "callback-provenance child failed");

            // ---- fatal containment + committed prefix (child: trips a
            // real MuJoCo fatal) ----
            bt::require(
                in_child([&] { committed_prefix_body(model_path); }),
                "fatal committed-prefix child failed");

            // ---- nested frames + thread-independent contexts (child:
            // raw + framed fatals across threads) ----
            bt::require(
                in_child([&] { nested_and_threads_body(model_path); }),
                "nested/threads child failed");
        });
    } catch (...) {
        return 1;
    }
}
