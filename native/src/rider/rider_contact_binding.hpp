// rider_contact_binding.hpp — the nanobind ADAPTER face for the rider
// contacts writer. The Python-free Value twin of the state decode
// (schema + writer-topology gates) is rider::decode_rider_contacts_state
// in rider/state_codec.hpp (T08); RiderContactsState/RiderContactsProbe
// remain owned by writers/rider_contacts.hpp.
#pragma once
#include <optional>
#include <utility>
#include <nanobind/nanobind.h>
#include "../writers/rider_contacts.hpp"
class Stepper;

// T3b-1 seam: the model-owned RiderContactWriter's Stepper method face —
// qfrc evaluation, lifecycle (reset/restart/settled-init), enable/release,
// diagnostics and the serializable state. Attachment settle/prepare/sample
// bindings are wave 3b and intentionally absent.
void bind_rider_contacts(const nanobind::module_ &module,
                         nanobind::class_<Stepper> &cls);

// Schema+topology decode of the rider-contacts state dict — the same reader
// 'set_rider_contacts_state' uses. Pure: no writer or model mutation.
// The bike::io::Value twin is rider::decode_rider_contacts_state.
[[nodiscard]] std::pair<writers::RiderContactsState,
                        std::optional<writers::RiderContactsProbe>>
parse_rider_contacts_state(nanobind::handle raw,
                           const writers::RiderContactWriter &writer);
