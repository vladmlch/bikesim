#include "../src/drivetrain/pedaling.hpp"
#include "../src/engine_call.hpp"
#include "../src/engine_abi_312.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

namespace {
    using TestFunction = void (*)();

    struct TestCase {
        std::string_view name;
        TestFunction run;
    };

    void require(bool condition, std::string_view message) {
        if (!condition) throw std::runtime_error(std::string(message));
    }

    void test_human_crank_torque() {
        require(drivetrain::human_crank_torque(50., 0., .35) == 32.5,
                "human_crank_torque at phase zero");
        require(std::abs(drivetrain::human_crank_torque(50., std::numbers::pi / 2., .35) - 67.5) < 1e-12,
                "human_crank_torque at half turn");
    }

    void test_pedaling_policy_valid_transition() {
        const drivetrain::PedalingConfig config{
            .enabled = true,
            .coast_above_rpm = 110.,
            .resume_below_rpm = 80.,
            .stop_time_s = 1.,
            .coast_cadence_tau_s = 0.,
            .mash_cadence_rpm = 25.,
            .mash_torque_nm = 25.,
            .effort_slew_nm_s = 0.,
        };
        drivetrain::PedalingPolicy policy(config);
        const drivetrain::PedalingState state = policy.update(0., 0., 30., 20., .01);
        require(state.mode == "pedaling", "pedaling policy mode");
        require(state.effort_nm == 20., "pedaling policy effort");
        require(!state.target_phase_rad, "pedaling policy target phase");
    }

    struct FatalContext { const char *message; };
    // Operation uses the required mutable void* ABI despite reading its context.
    // NOLINTNEXTLINE(misc-const-correctness)
    void fatal_operation(void *raw) noexcept {
        const auto *context = static_cast<const FatalContext *>(raw);
        // This case exercises MuJoCo's real variadic fatal-error entry point.
        // Every context.message below names a NUL-terminated string literal.
#if defined(__clang__)
#pragma clang diagnostic push
#if __has_warning("-Wunsafe-buffer-usage-in-format-attr-call")
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-format-attr-call"
#endif
#endif
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg,clang-diagnostic-unsafe-buffer-usage-in-format-attr-call)
        mju_error("%s", context->message);
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    }

    void no_operation(void *) noexcept {}

    void test_engine_fatal_status_and_reuse() {
        engine::ErrorBuffer error;
        FatalContext context{.message = "contract fatal A"};
        require(!engine::invoke(&fatal_operation, &context, error),
                "fatal operation must return failure status");
        require(error.kind == engine::ErrorKind::fatal, "fatal status kind");
        require(std::string_view(error.message.data()).find(context.message) != std::string_view::npos,
                "fatal message survives C frames");
        require(engine::invoke(&no_operation, nullptr, error),
                "handler must be reusable after a fatal error");
        require(error.kind == engine::ErrorKind::none, "successful status clears fatal kind");
    }

    struct NestedContext { bool inner_failed = false; };
    void nested_operation(void *raw) noexcept {
        auto *context = static_cast<NestedContext *>(raw);
        engine::ErrorBuffer inner;
        FatalContext fatal{.message = "nested fatal"};
        context->inner_failed =
            !engine::invoke(&fatal_operation, &fatal, inner) &&
            inner.kind == engine::ErrorKind::fatal;
    }

    void test_nested_engine_frames() {
        NestedContext context;
        engine::ErrorBuffer outer;
        require(engine::invoke(&nested_operation, &context, outer),
                "outer engine operation survives inner fatal error");
        require(context.inner_failed, "inner fatal error reached its own frame");
    }

    void test_thread_local_engine_frames() {
        engine::ErrorBuffer first;
        engine::ErrorBuffer second;
        FatalContext first_context{.message = "thread fatal A"};
        FatalContext second_context{.message = "thread fatal B"};
        bool first_failed = false;
        bool second_failed = false;
        std::thread first_thread([&] {
            first_failed = !engine::invoke(&fatal_operation, &first_context, first);
        });
        std::thread second_thread([&] {
            second_failed = !engine::invoke(&fatal_operation, &second_context, second);
        });
        first_thread.join();
        second_thread.join();
        require(first_failed && second_failed, "both fatal errors must be caught");
        require(std::string_view(first.message.data()).find(first_context.message) != std::string_view::npos,
                "first thread must keep its own error");
        require(std::string_view(second.message.data()).find(second_context.message) != std::string_view::npos,
                "second thread must keep its own error");
    }

    // Thread-local counter observes forwarding without shared mutable state.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    thread_local int forwarded_warnings = 0;
    void warning_handler(const mjLogMessage *message) {
        if (message->level == mjLOG_WARNING) ++forwarded_warnings;
    }
    void warning_operation(void *) noexcept {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
        mju_warning("contract warning");
    }

    void test_previous_tls_handler_restored() {
        forwarded_warnings = 0;
        const mjfLogHandler original = _mjPRIVATE_setTlsLogHandler(&warning_handler);
        engine::ErrorBuffer error;
        const bool succeeded = engine::invoke(&warning_operation, nullptr, error);
        const mjfLogHandler restored = _mjPRIVATE_setTlsLogHandler(original);
        require(succeeded, "warning operation must succeed");
        require(forwarded_warnings == 1, "nonfatal warning must reach prior handler");
        require(restored == &warning_handler, "prior TLS handler must be restored");
    }

    void *fail_allocation(std::size_t) noexcept { return nullptr; }
    struct AllocationFailureContext {
        const char *path = nullptr;
        const mjModel *model = nullptr;
        mjModel *loaded = nullptr;
        mjData *data = nullptr;
    };
    void load_with_failed_allocation(void *raw) noexcept {
        auto *context = static_cast<AllocationFailureContext *>(raw);
        mju_user_malloc = &fail_allocation;
        context->loaded = mj_loadModel(context->path, nullptr);
    }
    void make_data_with_failed_allocation(void *raw) noexcept {
        auto *context = static_cast<AllocationFailureContext *>(raw);
        mju_user_malloc = &fail_allocation;
        context->data = mj_makeData(context->model);
    }

    int run_allocation_failure(const char *path, bool during_make_data) {
        try {
            const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                during_make_data ? engine::load_model(path) : nullptr,
                &mj_deleteModel);
            if (during_make_data) require(model != nullptr, "valid control model load");
            AllocationFailureContext context{
                .path = path, .model = model.get()
            };
            engine::ErrorBuffer error;
            const auto previous_allocator = mju_user_malloc;
            const bool succeeded = engine::invoke(
                during_make_data ? &make_data_with_failed_allocation
                                 : &load_with_failed_allocation,
                &context, error);
            mju_user_malloc = previous_allocator;
            if (context.data) mj_deleteData(context.data);
            if (context.loaded) mj_deleteModel(context.loaded);
            require(!succeeded, "MuJoCo allocation failure must be intercepted");
            require(error.kind == engine::ErrorKind::fatal,
                    "allocation failure must have fatal status");
            require(std::string_view(error.message.data()).find("Could not allocate memory") !=
                        std::string_view::npos,
                    "allocation failure must preserve engine message");
            std::cout << "PASS " << (during_make_data ? "makeData" : "loadModel")
                      << " fatal allocation\n";
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL fatal allocation: " << error.what() << '\n';
            return 1;
        }
    }

    struct LateAllocationState {
        int calls = 0;
        int outstanding = 0;
        std::array<void *, 8> pointers{};
    };
    // The process-isolated allocation probe installs this only inside invoke.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    thread_local LateAllocationState *late_allocation = nullptr;
    void *late_fail_allocation(std::size_t size) noexcept {
        auto &state = *late_allocation;
        if (++state.calls == 2) return nullptr;
        const std::size_t rounded = (size + 63U) & ~std::size_t{63U};
        // MuJoCo's allocator callback transfers ownership through void*.
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory)
        void *pointer = std::aligned_alloc(64, rounded);
        if (pointer != nullptr) {
            state.pointers[static_cast<std::size_t>(state.outstanding)] = pointer;
            ++state.outstanding;
        }
        return pointer;
    }
    void late_free_allocation(void *pointer) noexcept {
        if (pointer == nullptr) return;
        auto &state = *late_allocation;
        for (void *&owned : state.pointers) {
            if (owned == pointer) {
                owned = nullptr;
                --state.outstanding;
                break;
            }
        }
        // Paired with the raw allocator callback above, after MuJoCo releases it.
        // NOLINTNEXTLINE(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc)
        std::free(pointer);
    }
    struct RawAllocationContext { mjData *data; const mjModel *model; };
    void raw_data_with_late_failure(void *raw) noexcept {
        auto *context = static_cast<RawAllocationContext *>(raw);
        mju_user_malloc = &late_fail_allocation;
        mju_user_free = &late_free_allocation;
        mj_makeRawData(&context->data, context->model);
    }
    int run_late_data_allocation_failure(const char *path) {
        try {
            const std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                engine::load_model(path), &mj_deleteModel);
            require(model != nullptr, "valid control model load");
            auto *data = static_cast<mjData *>(mju_malloc(sizeof(mjData)));
            require(data != nullptr, "test mjData allocation");
            std::construct_at(data);
            LateAllocationState state;
            late_allocation = &state;
            RawAllocationContext context{.data = data, .model = model.get()};
            engine::ErrorBuffer error;
            const auto previous_malloc = mju_user_malloc;
            const auto previous_free = mju_user_free;
            const bool succeeded = engine::invoke(&raw_data_with_late_failure,
                                                  &context, error);
            mj_deleteData(data);
            mju_user_malloc = previous_malloc;
            mju_user_free = previous_free;
            late_allocation = nullptr;
            require(!succeeded && error.kind == engine::ErrorKind::fatal,
                    "late raw-data allocation must return fatal status");
            require(state.calls == 2 && state.outstanding == 0,
                    "owned raw-data buffers must be freed after late fatal");
            std::cout << "PASS late raw-data fatal allocation and cleanup\n";
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL late fatal allocation: " << error.what() << '\n';
            return 1;
        }
    }

    constexpr std::array<TestCase, 6> cases{{
        {.name = "human_crank_torque", .run = test_human_crank_torque},
        {.name = "pedaling_policy_valid_transition", .run = test_pedaling_policy_valid_transition},
        {.name = "engine_fatal_status_and_reuse", .run = test_engine_fatal_status_and_reuse},
        {.name = "nested_engine_frames", .run = test_nested_engine_frames},
        {.name = "thread_local_engine_frames", .run = test_thread_local_engine_frames},
        {.name = "previous_tls_handler_restored", .run = test_previous_tls_handler_restored},
    }};

    int run_case(const TestCase &test_case) {
        try {
            test_case.run();
            std::cout << "PASS " << test_case.name << '\n';
            return 0;
        } catch (const std::exception &error) {
            std::cerr << "FAIL " << test_case.name << ": " << error.what() << '\n';
            return 1;
        }
    }
} // namespace

int main(int argc, char **argv) {
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage-in-container"
#endif
    // The C process-entry contract provides argc pointers before the sentinel.
    const std::span<char *> arguments(argv, static_cast<std::size_t>(argc));
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
    if (arguments.size() == 2 && std::string_view(arguments[1]) == "--require-failure") {
        try {
            require(false, "deliberate require failure");
        } catch (const std::exception &error) {
            std::cerr << error.what() << '\n';
            return 1;
        }
        return 0;
    }
    if (arguments.size() == 2 && std::string_view(arguments[1]) == "--list") {
        for (const TestCase &test_case : cases) std::cout << test_case.name << '\n';
        return 0;
    }
    if (arguments.size() == 3 &&
        std::string_view(arguments[1]) == "--load-allocation-failure")
        return run_allocation_failure(arguments[2], false);
    if (arguments.size() == 3 &&
        std::string_view(arguments[1]) == "--make-data-allocation-failure")
        return run_allocation_failure(arguments[2], true);
    if (arguments.size() == 3 &&
        std::string_view(arguments[1]) == "--late-data-allocation-failure")
        return run_late_data_allocation_failure(arguments[2]);
    if (arguments.size() > 2) {
        std::cerr << "usage: native_contract_tests [case-name|--list|--require-failure]\n";
        return 2;
    }

    const std::string_view requested =
        arguments.size() == 1 ? "all" : std::string_view(arguments[1]);
    bool found = requested == "all";
    int status = 0;
    for (const TestCase &test_case : cases) {
        if (requested == "all" || requested == test_case.name) {
            found = true;
            if (run_case(test_case) != 0) status = 1;
        }
    }
    if (!found) {
        std::cerr << "unknown native contract case: " << requested << '\n';
        return 2;
    }
    return status;
}
