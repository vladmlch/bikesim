// Completion 3.33a — vtable anchor for the header-declared test doubles in
// research_reference_support.hpp. Compiled once by the
// bike_test_research_reference OBJECT library and carried into the
// CountingPolicy-instantiating targets (frontend, policy, recording) through
// $<TARGET_OBJECTS:...> so -Wweak-vtables keeps an out-of-line key function
// and the ODR sweep's combined relocatable link sees a single definition.
#include "research_reference_support.hpp"

namespace research_reference {
CountingPolicy::~CountingPolicy() = default;
}
