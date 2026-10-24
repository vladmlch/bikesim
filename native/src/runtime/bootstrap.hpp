// bootstrap.hpp — the owned t=0 state inventory (design doc section 3.1;
// typed decode boundary completed for T09).
//
// The bootstrap captures what a freshly initialized Python physical runtime
// owns: the compiled-model identity (sha256 digest + full dims), the flat
// mjSTATE_INTEGRATION vector, the mutable model coefficients Python physics
// writes into mjModel, the mechanical writer snapshots (drive, rider
// contacts, tire brush) and the controller/intent/runtime documents staged
// for their native consumers.
//
// Every section decodes through the same readers the Stepper bindings use —
// the bootstrap introduces no second wire format. All validation happens
// before any mutation: a rejected bootstrap leaves a constructed runtime
// untouched.
//
// Owned-typing contract: sections whose decode needs only the document and
// the loaded model arrive already decoded (digest, dims, integration
// vector, resolved model coefficients, tire, drive, step/generation).
// Sections whose decode position is contractually later than the staged
// parse — the writer-topology-bound rider_contacts state, the step-state
// inputs (rider_controller/rider_intent/signals) and the state.runtime
// document the accounting layer re-reads — stay owned bike::io::Value
// trees; the runtime constructor decodes them at the same ordinal
// positions the nanobind build did.
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>

#include "../io/value.hpp"
#include "../writers/drivetrain.hpp"
#include "../writers/rider_contacts.hpp"
#include "config.hpp"
#include "../spatial/bootstrap_state.hpp"
#include "../spatial/brake_state.hpp"
#include "../spatial/topology.hpp"
#include "../pneumatic3d/state.hpp"

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
    // Spatial continuation only: full dof/site/tendon/wrap rows in the fixed
    // order declared by the checkpoint codec; sparse entries stay empty.
    std::optional<std::array<std::vector<double>,7>> spatial_arrays;
};

// The captured model dimensions — validated against the live model when
// the document decodes and re-verified by the runtime ctor so a fabricated
// bootstrap cannot slip a mismatched record past ownership.
struct ModelDims {
    int nq = 0, nv = 0, nu = 0, na = 0;
    int nbody = 0, njnt = 0, ntendon = 0, nwrap = 0;
    int nsite = 0, ngeom = 0, neq = 0, nsensor = 0, nsensordata = 0;
};

struct TireStateRow {
    std::vector<std::string> names;
    std::vector<double> row;
};

// Causal owners share this record across capture, reset and initial-state
// transfer. The tyre has one separate typed material owner, never a row cast.
struct SpatialBootstrapState {
    bike::spatial::RootKind root_kind = bike::spatial::RootKind::constrained4;
    std::string topology_fingerprint;
    std::string terrain_sha256;
    std::string calibration_sha256;
    bike::spatial::BootstrapPhase bootstrap_phase = bike::spatial::BootstrapPhase::settling;
    bike::spatial::HoldState hold = bike::spatial::canonical_inactive_hold();
    bike::spatial::BrakeState brake = bike::spatial::canonical_released_brake();
    bike::spatial::SteerState steer = bike::spatial::canonical_inactive_steer();
    bike::spatial::OutcomeState outcome = bike::spatial::canonical_unlatched_outcome();
};

[[nodiscard]] SpatialBootstrapState decode_spatial_bootstrap(const bike::io::Value&);
[[nodiscard]] bike::io::Value encode_spatial_bootstrap(const SpatialBootstrapState&);
void validate_schema_family(const RuntimeConfig&, int bootstrap_schema);
void validate_spatial_identity(const RuntimeConfig&, const SpatialBootstrapState&,
                               const bike::spatial::Topology&);

struct BootstrapState {
    std::string model_digest;
    ModelDims model_dims;
    std::vector<double> integration_state;
    ModelMutableState model_mutable;
    TireStateRow tire;
    drivetrain::DriveSnapshot drive;
    // state.rider_contacts — null or the owned document; the runtime ctor
    // decodes it against the live RiderContactWriter topology.
    bike::io::Value rider_contacts;
    // state.rider_controller — null or the owned document (the spindle
    // controller state the step-state decode restores).
    bike::io::Value rider_controller;
    // state.rider_intent / state.signals — owned documents decoded into
    // the intent stack at step-state decode.
    bike::io::Value rider_intent;
    bike::io::Value signals;
    // state.runtime — the owned document the step-state and accounting
    // decoders consume at their contract positions.
    bike::io::Value runtime;
    // Scalars the runtime itself consumes, lifted out of `runtime`.
    std::int64_t step = 0;
    int generation = 0;
    int schema = kRuntimeSchema;
    std::optional<SpatialBootstrapState> spatial;
    std::optional<bike::pneumatic3d::TyreState3d> tire3d;
};

// Reads state.model_digest — validated presence + string type only. The
// runtime compares it against sha256(model bytes) before loading.
[[nodiscard]] std::string expected_model_digest(const bike::io::Value &state);

// Envelope validation + the decode halves that need only the document and
// the loaded model: schema version, model_dims == model counters,
// integration width == mj_stateSize(mjSTATE_INTEGRATION), name-resolved
// mutable coefficients, tire and drive snapshots, staged document
// extraction and the state.runtime surface (key set, step, generation).
// Throws std::invalid_argument naming the state path on any mismatch;
// mutates nothing.
[[nodiscard]] BootstrapState decode_bootstrap_state(const bike::io::Value &state,
                                                    const mjModel &model);

// Re-verifies a decoded model_dims record against a live model — the
// runtime ctor's guard against a fabricated RuntimeBootstrap. Throws
// std::invalid_argument naming state.model_dims.<field> on a mismatch.
void verify_model_dims(const ModelDims &dims, const mjModel &model);

// The runtime_schema=1 configuration envelope decode — the owned twin of
// the former binding-side parse_runtime_config. Throws std::invalid_argument
// naming the full config path on any deviation.
[[nodiscard]] RuntimeConfig decode_runtime_config(const bike::io::Value &config);

} // namespace runtime
