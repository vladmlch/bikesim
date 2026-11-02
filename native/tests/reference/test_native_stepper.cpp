// Completion 3.20/T52: C++ port of tests/reference/test_native_stepper.py.
// Native Stepper::step() is bitwise-identical to raw mj_step over the same
// compiled image — the oracle's 200-step qpos/qvel rows are frozen in the
// two-platform goldens; the parallel in-process mjModel/mjData run keeps
// the same-dylib claim exercised.
#include "tyre_reference_support.hpp"

#include "engine_call.hpp"
#include "model_access.hpp"
#include "support/model_fixture.hpp"

namespace {

using namespace tyre_reference;

[[gnu::noinline]] void test_native_step_matches_python_bitwise() {
    auto golden = node("test_native_stepper.py",
                       "test_native_step_matches_python_bitwise");
    const auto path =
            repo_file("tests/fixtures/benchmark/model.mjb");
    Stepper native(path.string());
    const auto image = fixture_bytes(path);
    bike::model::ModelOwner ref(engine::load_model_image(image));
    std::unique_ptr<mjData, bt::DataDeleter> data(
            engine::make_data(ref.get()));
    bt::require(data != nullptr, "MuJoCo test fixture owns its data");
    engine::forward(ref.get(), data.get());
    for (int i = 0; i < 200; ++i) {
        native.step();
        engine::step(ref.get(), data.get());
    }
    // The live comparison doubles as the construction proof; the
    // authoritative gate is the recorded oracle entry.
    const std::span<const mjtNum> ref_qpos =
            std::views::counted(data->qpos, ref->nq);
    const std::span<const mjtNum> ref_qvel =
            std::views::counted(data->qvel, ref->nv);
    bt::require(std::ranges::equal(std::as_bytes(native.qpos()),
                                   std::as_bytes(ref_qpos)),
                "native qpos matches raw mj_step bitwise");
    bt::require(std::ranges::equal(std::as_bytes(native.qvel()),
                                   std::as_bytes(ref_qvel)),
                "native qvel matches raw mj_step bitwise");
    golden.f64(native.qpos(),
               std::array<std::size_t, 1>{native.qpos().size()}, "qpos");
    golden.f64(native.qvel(),
               std::array<std::size_t, 1>{native.qvel().size()}, "qvel");
    golden.done();
}

} // namespace

int main() {
    return bt::run([] { test_native_step_matches_python_bitwise(); });
}
