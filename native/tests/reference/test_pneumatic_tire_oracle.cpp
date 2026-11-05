#include "pneumatic_tire_fixture.hpp"
#include "support/golden.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>

namespace {
using namespace bt::pneumatic;
void scalar(bt::oracle::Node &node, double value, std::string_view label) {
    const std::array actual{value};
    node.f64(actual, std::span<const std::size_t>{}, label);
}
void vector(bt::oracle::Node &node, std::span<const double> values, std::string_view label) {
    const std::array<std::size_t, 1> shape{values.size()};
    node.f64(values, shape, label);
}
void keys(bt::oracle::Node &node, const biketyre::PneumaticTyreModel &wheel, std::string_view label) {
    const auto ordered = patch_keys(wheel);
    std::vector<std::byte> bytes;
    // Explicit little-endian int64, including the (0, 2) empty key set.
    for (const auto &[start, stop] : ordered)
        for (const auto integer : {static_cast<std::int64_t>(start), static_cast<std::int64_t>(stop)}) {
            const auto bits = std::bit_cast<std::uint64_t>(integer);
            for (unsigned shift = 0; shift < 64; shift += 8)
                bytes.push_back(static_cast<std::byte>((bits >> shift) & UINT64_C(255)));
        }
    const std::array<std::size_t, 2> shape{ordered.size(), 2};
    node.bytes(bytes, "<i8", shape, label);
}

// pneumatic_tire_forces/oracle_parity_with_xfrc_writer/tier — the qfrc
// writer's carcass, patch and output state per step, and qfrc equal to the
// sum of the patch wrenches mapped through the Jacobian. The legacy
// xfrc-writer parity half was removed with legacy mode (2026-11-04).
void test_oracle_parity_with_xfrc_writer(std::string_view tier) {
    bt::oracle::Node node("pneumatic_tire_forces/oracle_parity_with_xfrc_writer/" + std::string(tier));
    Rig rig(tier);
    // The removed legacy applier rewrote the model timestep to the tier's
    // own step; dt is that step, not the rig's compile-time default.
    const double dt = bt::tyres::tier_spec(tier).timestep_s;
    rig.spin("front", 2.);
    for (int step = 0; step < 4; ++step) {
        const auto actual = rig.tire->qfrc(rig.engine.data.get(), dt);
        std::vector<double> expected(actual.size(), 0.);
        for (std::size_t side = 0; side < sides.size(); ++side) {
            const auto prefix = "step." + std::to_string(step) + "." + std::string(sides[side]);
            const auto &wheel = Access::wheel(*rig.tire, side);
            const auto &state = wheel.carcass_state();
            vector(node, state.delta_prev_m(), prefix + ".delta_prev_m");
            vector(node, state.elastic_prev_n(), prefix + ".elastic_prev_n");
            vector(node, state.maxwell_force_n(), prefix + ".maxwell_force_n");
            scalar(node, state.initialized() ? 1. : 0., prefix + ".initialized");
            keys(node, wheel, prefix + ".patch_keys");
            for (const auto &[start, stop] : patch_keys(wheel)) {
                const auto &patch = patch_at(wheel, start, stop);
                const auto label = prefix + ".patch." + std::to_string(start) + "." + std::to_string(stop);
                scalar(node, patch.kappa_prime, label + ".kappa_prime");
                if (patch.has_detailed)
                    vector(node, patch.bristle_deflection_m, label + ".bristle_deflection_m");
            }
            const auto output = wheel.last_outputs().patches();
            scalar(node, static_cast<double>(output.size()), prefix + ".output_patch_count");
            for (std::size_t i = 0; i < output.size(); ++i) {
                const auto label = prefix + ".output." + std::to_string(i);
                vector(node, output[i].force_world_n, label + ".force_world_n");
                scalar(node, output[i].slip_speed_mps, label + ".slip_speed_mps");
                add_jtf(*rig.engine.model, *rig.engine.data, sides[side], output[i].centroid_world_m,
                    output[i].force_world_n, expected);
            }
        }
        vector(node, actual, "step." + std::to_string(step) + ".qfrc");
        numeric_equal(actual, expected);
        rig.engine.data->time += dt;
    }
    node.done();
}
} // namespace
// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// cppcheck-suppress throwInEntryPoint
int main() { return bt::run([] {
    for (const auto tier : bt::tyres::kTiers) test_oracle_parity_with_xfrc_writer(tier);
}); }
