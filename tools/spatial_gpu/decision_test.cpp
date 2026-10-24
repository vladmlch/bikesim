// Source-authored W55 controls; build/run only in the optional V11 lane.
#include "decision.hpp"
#include "support/migration_test.hpp"

#include <array>
#include <cmath>
#include <limits>

int main() {
    return bt::run([] {
        using namespace spatial_gpu;
        bt::require(!latency_goal_met(10., 2., false, true), "speed is not accuracy");
        bt::require(!latency_goal_met(10., 2., true, false), "speed is not determinism");
        bt::require(latency_goal_met(10., 5., true, true), "inclusive twofold goal");
        bt::require(!latency_goal_met(10., 5.01, true, true), "twofold goal has no tolerance widening");
        bt::invalid([] { (void)latency_goal_met(0., 1., true, true); });
        bt::require(batch_goal_met(1., 20., true), "inclusive twentyfold batch goal");
        bt::require(!batch_goal_met(1., 19.9, true), "batch threshold is not rounded");
        bt::require(!batch_goal_met(1., 100., false), "different physics cannot qualify throughput");
        bt::require(offline_goal_met(3., 1., true), "inclusive threefold offline goal");
        bt::require(!offline_goal_met(3., 1.01, true), "offline threshold is not rounded");
        bt::require(!offline_goal_met(3., 0.5, false), "offline artifacts must pass their exact checks");

        bt::require(interactive_goal_met(10., 5., 100., 90., true, true),
                    "H1 requires inclusive section and full-ride gains together");
        bt::require(!interactive_goal_met(10., 1., 100., 91., true, true),
                    "fast shader without ten percent whole-ride gain is no-go");
        bt::require(!interactive_goal_met(10., 5.01, 100., 80., true, true),
                    "whole-ride gain does not override a missed section target");
        bt::require(!interactive_goal_met(10., 2., 100., 80., false, true),
                    "H1 full-ride gain cannot override accuracy");

        constexpr std::array<double, 5> invalid{
            0., -1., std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN()};
        for (const double value : invalid) {
            bt::invalid([&] { (void)latency_goal_met(value, 1., true, true); });
            bt::invalid([&] { (void)latency_goal_met(1., value, false, false); });
            bt::invalid([&] { (void)batch_goal_met(value, 1., true); });
            bt::invalid([&] { (void)batch_goal_met(1., value, false); });
            bt::invalid([&] { (void)offline_goal_met(value, 1., true); });
            bt::invalid([&] { (void)offline_goal_met(1., value, false); });
            bt::invalid([&] { (void)interactive_goal_met(1., 0.5, value, 1., true, true); });
            bt::invalid([&] { (void)interactive_goal_met(1., 0.5, 1., value, false, false); });
        }
        const double maximum = std::numeric_limits<double>::max();
        bt::require(!batch_goal_met(maximum, maximum, true),
                    "overflow-sized throughput target cannot spuriously qualify");
        bt::require(!offline_goal_met(maximum, maximum, true),
                    "overflow-sized offline target cannot spuriously qualify");
        const double next = std::nextafter(5., 6.);
        bt::require(!latency_goal_met(10., next, true, true),
                    "one representable step above the latency boundary is rejected");
        const double unit = std::numeric_limits<double>::denorm_min();
        bt::require(!latency_goal_met(3. * unit, 2. * unit, true, true),
                    "rounding an odd subnormal baseline cannot invent a twofold section gain");
        bt::require(latency_goal_met(4. * unit, 2. * unit, true, true),
                    "the exact twofold subnormal section boundary still qualifies");
        bt::require(!interactive_goal_met(unit, unit, unit, unit, true, true),
                    "identical subnormal section and ride times cannot qualify");
        bt::require(!interactive_goal_met(2. * unit, unit, 11. * unit, 10. * unit, true, true),
                    "subnormal ride rounding cannot turn a nine-percent gain into ten percent");
        bt::require(interactive_goal_met(2. * unit, unit, 10. * unit, 9. * unit, true, true),
                    "exact ten-percent full-ride gain remains inclusive for finite subnormal inputs");
    });
}
