// Held-GIL boxing only. The accounting and sample cores do not depend on Python.
#pragma once
#include <nanobind/nanobind.h>
#include "samples.hpp"
namespace runtime {
[[nodiscard]] nanobind::object wire_to_python(const Wire &value);
[[nodiscard]] nanobind::dict wire_object_to_python(const WireObject &value);
[[nodiscard]] nanobind::dict columns_to_python(const SampleColumns &columns);
void bind_samples(nanobind::module_ &module);
} // namespace runtime
