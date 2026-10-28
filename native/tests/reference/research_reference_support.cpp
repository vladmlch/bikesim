// Completion 3.33a — vtable anchor for the header-declared test doubles in
// research_reference_support.hpp. Compiled into every native_research_*
// target alongside its test TU so -Wweak-vtables keeps an out-of-line key
// function.
#include "research_reference_support.hpp"

namespace research_reference {
CountingPolicy::~CountingPolicy() = default;
}
