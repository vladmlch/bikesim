#pragma once
#include <nanobind/nanobind.h>
class Stepper;

// T3b-1 seam: the model-owned RiderContactWriter's Stepper method face —
// qfrc evaluation, lifecycle (reset/restart/settled-init), enable/release,
// diagnostics and the serializable state. Attachment settle/prepare/sample
// bindings are wave 3b and intentionally absent.
void bind_rider_contacts(const nanobind::module_ &module,
                         nanobind::class_<Stepper> &cls);
