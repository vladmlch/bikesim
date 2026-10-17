// T26: exact representable controls for source arithmetic, with the existing
// captured state codec as an immutable structural oracle. Broader bitwise
// trace capture/qualification remains a separately recorded execution gate.
#include "kernel_test_support.hpp"
#include "support/migration_test.hpp"
#include "drivetrain/chain.hpp"
#include "drivetrain/freehub.hpp"
#include "drivetrain/motor.hpp"
#include "drivetrain/state_codec.hpp"
#include "io/json.hpp"

#include <bit>
#include <cstdint>
#include <string>

namespace {
void exact(double actual, double expected, const char* label) {
    bt::require(std::bit_cast<std::uint64_t>(actual) ==
                std::bit_cast<std::uint64_t>(expected), label);
}

void exact_controls() {
    // All inputs/results here are dyadic values. These are independently
    // derived arithmetic controls, not newly generated source goldens.
    const auto spring = drivetrain::chain_tension(.125, 2., 64., 4.);
    exact(spring.first, 16., "chain spring+damping exact arithmetic");
    exact(spring.second, .5, "chain elastic energy exact arithmetic");
    drivetrain::Freehub hub(64., 0.);
    exact(hub.update(0., 0., 0., 0.), 0., "freehub initial boundary");
    exact(hub.update(.25, 0., 0., 0.), 16., "freehub engagement torque");
    exact(hub.state().energy_j, 2., "freehub engagement energy");
    exact(hub.update(-.5, 0., 0., 0.), 0., "overrun moves the one-way boundary");

    drivetrain::Battery battery(16.);
    const auto debit = battery.debit(12., .25);
    exact(battery.state().energy_j, 16., "debit stages battery energy");
    exact(debit.second.energy_j, 13., "debit candidate energy");
    exact(debit.second.drawn_energy_j, 3., "debit accounts delivered energy");
    battery.publish(debit.second);
    battery.reset();
    exact(battery.state().energy_j, 16., "reset restores battery capacity");
    exact(drivetrain::motor_electrical_power(4., 2., .5, .25, 1., true),
          18., "shaft, copper, speed and idle power remain separate terms");
    exact(drivetrain::motor_electrical_power(4., 2., .5, .25, 1., false),
          0., "disabled motor consumes no synthetic idle power");

    const auto source = bt::text(bt::fixture("state/drive.json"));
    const auto decoded = drivetrain::decode_drive_snapshot(bike::io::parse_json(source));
    bt::golden_text("state/drive.canonical.json",
        bike::io::canonical_json(drivetrain::encode_drive_snapshot(decoded)));
}
} // namespace

int main() { return bt::run([] { exact_controls(); kernel_properties(); }); }
