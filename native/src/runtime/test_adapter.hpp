// runtime/test_adapter.hpp — the native-only parity surface (plan A2).
//
// NativeTestAdapter owns a Stepper and the optional subsystem probes the
// A2 tests compare against the Python oracle. It is deliberately separate
// from NativeRideRuntime: probes may inspect or mutate state the
// production adapter never exposes.
#pragma once
#include <nanobind/nanobind.h>

namespace runtime {

void bind_test_adapter(const nanobind::module_ &module);

} // namespace runtime
