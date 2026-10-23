#include "support/planar_bridge_cases.hpp"

int main() {
    return bt::run([] { spatial_tests::bridge::run(10., false); });
}
