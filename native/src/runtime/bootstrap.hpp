// bootstrap.hpp — the owned t=0 state inventory (design doc section 3.1).
//
// The bootstrap captures what a freshly initialized Python physical runtime
// owns: the compiled-model identity (sha256 digest + full dims), the flat
// mjSTATE_INTEGRATION vector, the mutable model coefficients Python physics
// writes into mjModel, the mechanical writer snapshots (drive, rider
// contacts, tire brush) and the controller/intent/runtime scalars staged
// for their native consumers.
//
// Every section decodes through the same readers the Stepper bindings use —
// the bootstrap introduces no second wire format. All validation happens
// before any mutation: a rejected bootstrap leaves a constructed runtime
// untouched.
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <nanobind/nanobind.h>
#include "../drivetrain/drive_binding.hpp"
#include "../rider/rider_contact_binding.hpp"

class Stepper;

namespace runtime {

// A mutable model coefficient resolved to its flat mjModel offset during
// parse — applying it is a pure write with no name lookups left to fail.
struct FrictionLossEntry {
    std::string joint;
    int dof_index;
    int dof_adr;
    double value;
};

struct SitePosEntry {
    std::string site;
    int site_id;
    std::array<double, 3> value;
};

struct ModelMutableState {
    std::vector<FrictionLossEntry> dof_frictionloss;
    std::vector<SitePosEntry> site_pos;
};

struct TireStateRow {
    std::vector<std::string> names;
    std::vector<double> row;
};

struct BootstrapState {
    std::string model_digest;
    std::vector<double> integration_state;
    ModelMutableState model_mutable;
    TireStateRow tire;
    drivetrain::DriveSnapshot drive;
    std::optional<std::pair<writers::RiderContactsState,
                            std::optional<writers::RiderContactsProbe>>>
        rider_contacts;
    // Owned plain-data trees staged for their A2/A3 native consumers.
    nanobind::object rider_controller;
    nanobind::object rider_intent;
    nanobind::object signals;
    nanobind::object runtime;
    // Scalars the runtime itself consumes, lifted out of `runtime`.
    std::int64_t step = 0;
    int generation = 0;
};

// Reads state.model_digest — validated presence + string type only. The
// runtime compares it against sha256(model_path bytes) before loading.
[[nodiscard]] std::string expected_model_digest(nanobind::handle raw_state);

// Full decode: schema version, model_dims == model counters, integration
// width == mj_stateSize(mjSTATE_INTEGRATION), name-resolved mutable
// coefficients, and the writer/tire/state sections. `stepper` supplies the
// writer topology the rider-contacts decoder needs. Throws
// std::invalid_argument naming the state path on any mismatch; performs no
// mutation of the stepper.
[[nodiscard]] BootstrapState parse_bootstrap(nanobind::handle raw_state,
                                             const mjModel &model,
                                             const Stepper &stepper);

} // namespace runtime
