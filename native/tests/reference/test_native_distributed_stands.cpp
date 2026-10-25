// test_native_distributed_stands.cpp — C++ port of
// tests/reference/test_native_distributed_stands.py (batch 3.13).
//
// The contact_manifold_rigs flat/step wheel-stand cells on the native
// distributed writer: the rig's own bounds (energy ledger, contact
// count, force error, multi-patch count) asserted as literals — the
// metrics come from the shared validation rig pipeline.
#include "tyre_reference_support.hpp"

#include "validation/rigs.hpp"

namespace {

using namespace tyre_reference;
using bike::validation::get;
using bike::validation::real;

// tests/reference/test_native_distributed_stands.py::test_distributed_flat_stand_native
// distributed_flat_rig's cell (dt=.000625, 128 stations, 600 N) — the
// rig's own bounds.
void test_distributed_flat_stand_native() {
    const auto result = bike::validation::distributed_flat_rig(
            .000625, 128, 600.);
    bt::require(real(result.metrics, "force_relative_error") <= .02,
                "flat stand force error inside the rig bound");
    bt::require(real(result.metrics, "native_wheel_contact_count") == 0.,
                "no MuJoCo wheel contacts");
    bt::require(real(result.metrics, "energy_residual_ratio") <= .02,
                "flat stand energy residual inside the rig bound");
}

// tests/reference/test_native_distributed_stands.py::test_distributed_step_stand_native
// distributed_step_rig's cell (dt=.000625, 128 stations, dx=.005) —
// energy and multi-patch bounds.
void test_distributed_step_stand_native() {
    const auto result = bike::validation::distributed_step_rig(
            .000625, 128, .005);
    bt::require(real(result.metrics, "native_wheel_contact_count") == 0.,
                "no MuJoCo wheel contacts");
    bt::require(real(result.metrics, "energy_residual_ratio") <= .05,
                "step stand energy residual inside the rig bound");
    bt::require(real(result.metrics, "max_patch_count") >= 2.,
                "step stand produces multi-patch intervals");
}

} // namespace

int main() {
    return bt::run([] {
        test_distributed_flat_stand_native();
        test_distributed_step_stand_native();
    });
}
