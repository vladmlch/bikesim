// Immutable identity compiled into the selected extension, never a sidecar claim.
#pragma once
#include <nanobind/nanobind.h>
namespace runtime {
void bind_provenance(nanobind::module_ &module);
} // namespace runtime
