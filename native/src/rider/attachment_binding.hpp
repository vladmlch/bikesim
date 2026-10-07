#pragma once
#include <nanobind/nanobind.h>
class Stepper;

// T2a/T2b seam: the Stepper method face (rider_equality_qfrc) and every
// module-level diagnostic hook the attachment port exposes. The T2b
// attachment_wrench.* bindings extend this single entry point rather
// than touching binding.cpp.
void bind_rider_attachment(nanobind::module_ &module,
                           nanobind::class_<Stepper> &cls);
