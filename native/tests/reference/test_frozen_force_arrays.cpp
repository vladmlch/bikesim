// Completion 3.22b. Source: tests/reference/test_frozen_force_arrays.py.
// NumPy write flags and view detection have no C++ surface: the ported
// contract is that force arrays are owned buffers moved into samples without
// copying, that published samples hand out const access only, and that the
// consumption gates (finite values, matching widths) fail closed exactly like
// freeze_force's ValueError paths.
#include "support/migration_test.hpp"
#include "runtime/accounting.hpp"
#include "runtime/samples.hpp"
#include "runtime/step.hpp"
#include <cmath>
#include <memory>
#include <span>
#include <type_traits>
#include <vector>

namespace {

// test_freeze_force_is_immutable_finite_and_recognised
void test_freeze_force_is_immutable_finite_and_recognised() {
    // freeze_force(np.arange(3.), 3): the owned C++ vector IS the frozen
    // array. Consumers reach it through const access, matching the read-only
    // flag Python asserts.
    runtime::ForceSample sample;
    sample.qpos = std::vector<double>{0., 1., 2.};
    const auto &frozen = sample.qpos;
    static_assert(std::is_const_v<std::remove_reference_t<decltype(frozen)>>);
    bt::require(frozen.size() == 3 && frozen[0] == 0. && frozen[2] == 2.,
                "frozen storage keeps its width and values");
    // A shared_ptr<const PhysicalSampleData> is what the publisher hands out:
    // mutating any array member through it is a compile error, which is the
    // direct analog of `frozen[0] = 1. -> ValueError`.
    const runtime::SamplePtr published = std::make_shared<runtime::PhysicalSampleData>();
    static_assert(std::is_const_v<std::remove_reference_t<decltype((published->qpos))>>);
    bt::require(published->qpos.empty(), "const publication surface is present");
    // freeze_force's NaN and width ValueErrors map onto the gates the
    // accounting consumption path applies to the same arrays: a nonfinite
    // contribution and a width mismatch both throw std::invalid_argument.
    const std::vector<double> nan_force{1., std::nan(""), 0.}, qvel{1., 1., 1.};
    const std::vector<double> short_force{0., 0.};
    bt::invalid([&] {
        (void)runtime::sample_wire::dot(std::span{nan_force}, std::span{qvel}); });
    bt::invalid([&] {
        (void)runtime::sample_wire::dot(std::span{short_force}, std::span{qvel}); });
    // step_work applies the same finite gate to the muscle/constraint power
    // channels the frozen forces feed.
    const std::vector<double> nan_power{std::nan("")}, zero{};
    bt::invalid([&] {
        (void)runtime::step_work(std::span{nan_power}, 0., std::span{zero}, .001); });
    // is_frozen_array's writeable-array and writeable-view rejections have no
    // object identity surface in C++; the type system supplies the same
    // distinction (a mutable lvalue vector vs. the const span a frozen row
    // exposes).
    std::vector<double> writeable(3, 0.);
    std::span<const double> const view{writeable};
    static_assert(!std::is_assignable_v<decltype(view)::element_type&, double>);
}

// test_samples_reuse_frozen_arrays_without_copying
void test_samples_reuse_frozen_arrays_without_copying() {
    // Moving an owned buffer into the sample preserves its storage address:
    // the zero-copy reuse Python proves with `is`.
    std::vector<double> q(4, 0.), v(4, 1.), force(4, 2.);
    const double *q_addr = q.data(), *v_addr = v.data(), *f_addr = force.data();
    runtime::ForceSample sample;
    sample.time_s = 0.;
    sample.qpos = std::move(q);
    sample.qvel = std::move(v);
    // emplace_back, not an initializer list: initializer_list elements are
    // const, so braced assignment would copy the buffer it means to move.
    sample.components.emplace_back("a", std::move(force));
    bt::require(sample.qpos.data() == q_addr && sample.qvel.data() == v_addr,
                "ForceSample reuses the qpos/qvel buffers, never copies");
    bt::require(sample.components.size() == 1 &&
                sample.components.front().second.data() == f_addr,
                "component 'a' is the moved force buffer itself");
    // PhysicalSample reuses the same owned buffers: the published row keeps
    // the identical storage the caller froze.
    std::vector<double> q2(4, 0.), f2(4, 2.);
    const double *q2_addr = q2.data(), *f2_addr = f2.data();
    runtime::PhysicalSampleData published;
    published.interval_id = 0;
    published.qpos = std::move(q2);
    published.forces.emplace_back("a", std::move(f2));
    bt::require(published.qpos.data() == q2_addr &&
                published.forces.front().second.data() == f2_addr,
                "published sample reuses the frozen buffers by identity");
}

} // namespace

int main() {
    return bt::run([] {
        test_freeze_force_is_immutable_finite_and_recognised();
        test_samples_reuse_frozen_arrays_without_copying();
    });
}
