// Completion 3.33a — vtable anchor for the header-declared test doubles in
// research_reference_support.hpp. Compiled into the CountingPolicy-
// instantiating targets (frontend, policy, recording) alongside their test
// TUs so -Wweak-vtables keeps an out-of-line key function.
#include "research_reference_support.hpp"

namespace research_reference {
CountingPolicy::~CountingPolicy() = default;
}
