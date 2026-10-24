// bootstrap.cpp — the owned config/state document decoders (task T09).
//
// These are the mechanical ports of the former binding-side
// parse_runtime_config / parse_bootstrap / expected_model_digest onto the
// bike::io::Value readers in value_readers.hpp. Every ordinal position is
// preserved: the envelope validates first, then the artifact digest
// comparison, then the model-bound sections (dims, integration vector,
// resolved mutable coefficients), then the document-level sections (tire,
// drive), then the staged subdocuments and the state.runtime surface
// checks — the writer-bound rider-contacts decode stays in the runtime
// ctor where its live-topology input exists.
#include "bootstrap.hpp"
#include "value_readers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <mujoco/mujoco.h>

#include "../drivetrain/state_codec.hpp"
#include "../engine_call.hpp"
#include "../writer_config.hpp"
#include "../rider/state_codec.hpp"
#include "../terrain/source3d.hpp"
#include "setup_values.hpp"
#include <memory>

namespace runtime {

namespace {

namespace bio = bike::io;

// ---- shared field readers ----------------------------------------------

[[nodiscard]] double positive_field(const vread::Dict &d, const char *key) {
    const double value =
        vread::finite_real(vread::field(d, key), d.child(key));
    if (!(value > 0.)) vread::invalid(d.child(key), "expected positive value");
    return value;
}

[[nodiscard]] double nonnegative_field(const vread::Dict &d, const char *key) {
    const double value =
        vread::finite_real(vread::field(d, key), d.child(key));
    if (value < 0.) vread::invalid(d.child(key), "expected nonnegative value");
    return value;
}

[[nodiscard]] int positive_int_field(const vread::Dict &d, const char *key) {
    const int value = vread::integer32(vread::field(d, key), d.child(key));
    if (value <= 0)
        vread::invalid(d.child(key), "expected positive integer");
    return value;
}

[[nodiscard]] runtime::JointReference joint_reference(const vread::Dict &d,
                                                    const char *key) {
    const vread::Dict ref = vread::section(d, key);
    vread::exact(ref, vread::keys("joint", "dof_index"));
    runtime::JointReference out{
        .joint = vread::string(vread::field(ref, "joint"), ref.child("joint")),
        .dof_index = vread::integer32(vread::field(ref, "dof_index"),
                                     ref.child("dof_index"))};
    if (out.dof_index < 0)
        vread::invalid(ref.child("dof_index"), "expected nonnegative index");
    return out;
}

[[nodiscard]] std::optional<std::string> optional_string_field(
    const vread::Dict &d, const char *key) {
    const vread::Node value = vread::field(d, key);
    if (vread::is_none(value)) return std::nullopt;
    return vread::string(value, d.child(key));
}

// Required mapping field — the owned document the config stores verbatim
// (today's owned_mapping: the mapping type check runs at this position).
[[nodiscard]] bike::io::Value owned_mapping(const vread::Dict &d,
                                            const char *key) {
    const auto &source =
        vread::mapping(vread::field(d, key), d.child(key));
    return bike::io::Value{bio::Value::Object(source)};
}

// Optional mapping field — null stays a null Value (today's owned_optional:
// the mapping type check runs only on a present non-null member).
[[nodiscard]] bike::io::Value owned_optional(const vread::Dict &d,
                                             const char *key) {
    const vread::Node value = vread::field(d, key);
    if (vread::is_none(value)) return bike::io::Value{};
    return bike::io::Value{bio::Value::Object(
        vread::mapping(value, d.child(key)))};
}

// ---- bootstrap section decoders -----------------------------------------
// Each decoder owns its vread::Dict temporaries; keeping them in separate
// functions holds every frame under the 8 KiB guard.

// The (field name, live counter, record member) triples both the decode
// and the ctor-side re-verification share.
[[nodiscard]] std::array<std::pair<std::string_view,
                                   std::pair<int, int ModelDims::*>>, 13>
dims_fields(const mjModel &model) {
    return {{
        {"nq", {model.nq, &ModelDims::nq}},
        {"nv", {model.nv, &ModelDims::nv}},
        {"nu", {model.nu, &ModelDims::nu}},
        {"na", {model.na, &ModelDims::na}},
        {"nbody", {model.nbody, &ModelDims::nbody}},
        {"njnt", {model.njnt, &ModelDims::njnt}},
        {"ntendon", {model.ntendon, &ModelDims::ntendon}},
        {"nwrap", {model.nwrap, &ModelDims::nwrap}},
        {"nsite", {model.nsite, &ModelDims::nsite}},
        {"ngeom", {model.ngeom, &ModelDims::ngeom}},
        {"neq", {model.neq, &ModelDims::neq}},
        {"nsensor", {model.nsensor, &ModelDims::nsensor}},
        {"nsensordata", {model.nsensordata, &ModelDims::nsensordata}},
    }};
}

ModelDims decode_model_dims(const vread::Dict &root, const mjModel &model) {
    const vread::Dict dims = vread::section(root, "model_dims");
    vread::exact(dims,
                 vread::keys("nq", "nv", "nu", "na", "nbody", "njnt",
                             "ntendon", "nwrap", "nsite", "ngeom", "neq",
                             "nsensor", "nsensordata"));
    ModelDims out;
    // Same interleave as the nanobind reader: decode one field, compare it
    // against the live counter, then move to the next.
    for (const auto &[name, pair] : dims_fields(model)) {
        const std::string key(name);
        // wire::integer (int64 read), not integer32 — an out-of-int-range
        // dimension value mismatches instead of overflowing first.
        const std::int64_t expected =
            vread::integer(vread::field(dims, key), dims.child(key));
        out.*(pair.second) = static_cast<int>(expected);
        if (!std::cmp_equal(expected, pair.first))
            vread::invalid(dims.child(key), "model dimension mismatch");
    }
    return out;
}

std::vector<double> decode_integration_state(const vread::Dict &root,
                                             const mjModel &model) {
    std::vector<double> values = vread::vector(
        vread::field(root, "integration_state"), "state.integration_state");
    const mjtSize width =
        engine::state_size(&model, static_cast<int>(mjSTATE_INTEGRATION));
    if (!std::cmp_equal(values.size(), width))
        vread::invalid("state.integration_state",
                       "width does not match the model");
    return values;
}

ModelMutableState decode_model_mutable(const vread::Dict &root,
                                       const mjModel &model) {
    ModelMutableState out;
    const vread::Dict mutable_section = vread::section(root, "model_mutable");
    const bool spatial=vread::integer(vread::field(root,"schema"),"state.schema")==2;
    if(spatial) vread::exact(mutable_section,vread::keys("dof_frictionloss","site_pos"),vread::keys("spatial_arrays"));
    else vread::exact(mutable_section, vread::keys("dof_frictionloss", "site_pos"));
    if(spatial && bio::find_member(*mutable_section.value,"spatial_arrays")) {
        const auto rows=vread::section(mutable_section,"spatial_arrays");
        constexpr std::array names{"dof_frictionloss","site_pos","tendon_stiffness","tendon_damping",
            "tendon_lengthspring","tendon_range","wrap_prm"};
        vread::exact(rows,vread::keys("dof_frictionloss","site_pos","tendon_stiffness","tendon_damping",
            "tendon_lengthspring","tendon_range","wrap_prm"));
        const std::array<mjtSize,7> widths{model.nv,3*model.nsite,model.ntendon,model.ntendon,
            2*model.ntendon,2*model.ntendon,model.nwrap};
        out.spatial_arrays.emplace();
        for(std::size_t i=0;i<names.size();++i) {
            const auto& values=bio::as_array(*vread::field(rows,names[i]).value,rows.child(names[i]));
            if(std::cmp_not_equal(values.size(),widths[i]))vread::invalid(rows.child(names[i]),"model width mismatch");
            auto& destination=(*out.spatial_arrays)[i];destination.reserve(values.size());
            for(const auto& value:values)destination.push_back(vread::finite_real(vread::Node{&value},rows.child(names[i])));
        }
        if(!vread::section(mutable_section,"dof_frictionloss").value->empty() ||
           !vread::section(mutable_section,"site_pos").value->empty())
            vread::invalid(mutable_section.path,"full rows and sparse copies cannot both own model state");
    }
    const vread::Dict frictionloss =
        vread::section(mutable_section, "dof_frictionloss");
    // mjModel carries no per-joint dof count — it derives from jnt_type.
    const std::span<const int> joint_types =
        std::views::counted(model.jnt_type, model.njnt);
    const auto dof_count = [&joint_types](int joint) {
        switch (joint_types[static_cast<std::size_t>(joint)]) {
        case mjJNT_FREE: return 6;
        case mjJNT_BALL: return 3;
        default: return 1;
        }
    };
    const std::span<const int> dofadr =
        std::views::counted(model.jnt_dofadr, model.njnt);
    for (const auto &[side_name, item] : *frictionloss.value) {
        const vread::Dict entry{
            .value = &vread::mapping(vread::Node{&item},
                                     frictionloss.child(side_name)),
            .path = frictionloss.child(side_name)};
        vread::exact(entry, vread::keys("joint", "dof_index", "value"));
        FrictionLossEntry row{
            .joint = vread::string(vread::field(entry, "joint"),
                                   entry.child("joint")),
            .dof_index = vread::integer32(vread::field(entry, "dof_index"),
                                          entry.child("dof_index")),
            .dof_adr = 0,
            .value = vread::finite_real(vread::field(entry, "value"),
                                        entry.child("value"))};
        const int joint =
            mj_name2id(&model, mjOBJ_JOINT, row.joint.c_str());
        if (joint < 0)
            vread::invalid(entry.child("joint"), "unknown joint");
        if (row.dof_index < 0 ||
            row.dof_index >= dof_count(joint))
            vread::invalid(entry.child("dof_index"), "out of range");
        row.dof_adr = dofadr[static_cast<std::size_t>(joint)] +
                      row.dof_index;
        out.dof_frictionloss.push_back(std::move(row));
    }
    const vread::Dict site_pos = vread::section(mutable_section, "site_pos");
    for (const auto &[name, item] : *site_pos.value) {
        const int site = mj_name2id(&model, mjOBJ_SITE, name.c_str());
        if (site < 0)
            vread::invalid(site_pos.child(name), "unknown site");
        out.site_pos.push_back(SitePosEntry{
            .site = name, .site_id = site,
            .value = vread::fixed<3>(vread::Node{&item},
                                     site_pos.child(name))});
    }
    return out;
}

TireStateRow decode_tire_state(const vread::Dict &root) {
    TireStateRow out;
    const vread::Dict tire = vread::section(root, "tire");
    vread::exact(tire, vread::keys("names", "row"));
    for (const auto &value :
         vread::sequence(vread::field(tire, "names"), tire.child("names")))
        out.names.push_back(
            vread::string(vread::Node{&value}, tire.child("names")));
    // NaN marks unset brush columns — real, not finite_real.
    for (const auto &value :
         vread::sequence(vread::field(tire, "row"), tire.child("row")))
        out.row.push_back(
            vread::real(vread::Node{&value}, tire.child("row")));
    if (out.names.size() != out.row.size())
        vread::invalid(tire.child("row"), "names/row width mismatch");
    return out;
}

} // namespace

std::string expected_model_digest(const bike::io::Value &state) {
    const vread::Dict root{.value = &vread::mapping(vread::Node{&state},
                                                   "state"),
                           .path = "state"};
    return vread::string(vread::field(root, "model_digest"),
                         root.child("model_digest"));
}

RuntimeConfig decode_runtime_config(const bike::io::Value &config) {
    const vread::Dict root{.value = &vread::mapping(vread::Node{&config},
                                                   "config"),
                           .path = "config"};
    constexpr auto required = vread::keys(
        "runtime_schema", "timestep_s", "control_period_s",
        "control_period_steps", "physics_mode", "drive_mode", "strict",
        "measurement_decimation", "record_retention", "writer_config",
        "rider_controller", "rider_intent", "monitors", "geometry");
    const int version = vread::integer32(vread::field(root, "runtime_schema"),
                                         root.child("runtime_schema"));
    if (version == static_cast<int>(bike::spatial::kSpatialRuntimeSchema)) {
        vread::exact(root, required, vread::keys("async_telemetry", "spatial"));
        (void)vread::field(root, "spatial");
    } else vread::exact(root, required, vread::keys("async_telemetry"));
    if (version != kRuntimeSchema && version != static_cast<int>(bike::spatial::kSpatialRuntimeSchema))
        vread::invalid(root.child("runtime_schema"),
                       "unsupported runtime schema");

    RuntimeConfig out;
    out.schema = version;
    if (version == static_cast<int>(bike::spatial::kSpatialRuntimeSchema)) {
        out.spatial = bike::spatial::spatial_config_from_wire(*vread::field(root, "spatial").value);
        if (out.spatial->mode != bike::spatial::Mode::spatial3d)
            vread::invalid("config.spatial.mode", "runtime2 requires spatial mode");
    }
    out.timestep_s = positive_field(root, "timestep_s");
    out.control_period_s = positive_field(root, "control_period_s");
    out.control_period_steps =
        positive_int_field(root, "control_period_steps");
    if (std::abs(out.timestep_s * out.control_period_steps -
                 out.control_period_s) > 1e-9 * out.control_period_s)
        vread::invalid(root.child("control_period_s"),
                       "control_period_steps * timestep_s mismatch");
    if (vread::string(vread::field(root, "physics_mode"),
                      root.child("physics_mode")) != "physical")
        vread::invalid(root.child("physics_mode"), "unsupported physics mode");
    const auto& drive_mode = vread::string(vread::field(root, "drive_mode"),
                                          root.child("drive_mode"));
    if (drive_mode != "articulated_effort" && drive_mode != "crank_effort" &&
        drive_mode != "coast" && drive_mode != "ideal_speed_control")
        vread::invalid(root.child("drive_mode"), "unsupported drive mode");
    out.strict =
        vread::boolean(vread::field(root, "strict"), root.child("strict"));
    if (root.contains("async_telemetry"))
        out.async_telemetry = vread::boolean(vread::field(root, "async_telemetry"), root.child("async_telemetry"));
    out.measurement_decimation =
        positive_int_field(root, "measurement_decimation");
    {
        const std::string &mode = vread::string(
            vread::field(root, "record_retention"),
            root.child("record_retention"));
        if (mode == "off") out.record_retention = runtime::RetentionMode::off;
        else if (mode == "full") out.record_retention = runtime::RetentionMode::full;
        else if (mode == "legacy") out.record_retention = runtime::RetentionMode::legacy;
        else vread::invalid(root.child("record_retention"),
                            "expected 'off', 'full' or 'legacy'");
    }

    out.writer_config = owned_mapping(root, "writer_config");
    const auto writer = vread::section(root, "writer_config");
    const auto drive = vread::section(writer, "drive");
    if (vread::string(vread::field(drive, "drive_mode"), drive.child("drive_mode")) != drive_mode)
        vread::invalid(root.child("drive_mode"), "differs from writer_config.drive.drive_mode");
    out.rider_controller = owned_mapping(root, "rider_controller");
    out.rider_intent = owned_mapping(root, "rider_intent");

    const vread::Dict monitors = vread::section(root, "monitors");
    vread::exact(monitors,
                 vread::keys("balance_floor_mps", "balance_dwell_s",
                             "balance_grace_s", "grounded_hold_s"));
    out.monitors = MonitorConfig{
        .balance_floor_mps = positive_field(monitors, "balance_floor_mps"),
        .balance_dwell_s = positive_field(monitors, "balance_dwell_s"),
        .balance_grace_s = nonnegative_field(monitors, "balance_grace_s"),
        .grounded_hold_s = nonnegative_field(monitors, "grounded_hold_s")};

    const vread::Dict geometry = vread::section(root, "geometry");
    vread::exact(geometry,
                 vread::keys("crank_length_m", "brake_dofs", "wheel_bodies",
                             "com_marker_site", "pose"));
    out.geometry.crank_length_m = positive_field(geometry, "crank_length_m");
    const vread::Dict brake = vread::section(geometry, "brake_dofs");
    vread::exact(brake, vread::keys("front", "rear"));
    out.geometry.front_brake = joint_reference(brake, "front");
    out.geometry.rear_brake = joint_reference(brake, "rear");
    const vread::Dict bodies = vread::section(geometry, "wheel_bodies");
    vread::exact(bodies, {}, vread::keys("front", "rear"));
    out.geometry.front_wheel_body =
        bodies.contains("front")
            ? optional_string_field(bodies, "front")
            : std::nullopt;
    out.geometry.rear_wheel_body =
        bodies.contains("rear") ? optional_string_field(bodies, "rear")
                                : std::nullopt;
    out.geometry.com_marker_site =
        optional_string_field(geometry, "com_marker_site");
    out.geometry.pose = owned_optional(geometry, "pose");
    if (out.spatial) {
        validate_schema_family(out, bike::spatial::kSpatialBootstrapSchema);
        // Anatomical pose vectors are declared body-local 3-vectors. Root
        // world coordinates are exclusively the model-bound Topology owner.
        if (!bio::is_null(out.geometry.pose)) {
            (void)rider::decode_spatial_spindle_pose(out.geometry.pose, "config.geometry.pose");
            (void)rider::decode_spatial_spindle_config(out.rider_controller);
        } else (void)rider::decode_spindle_config(out.rider_controller);
        (void)rider::decode_intent_config(out.rider_intent, "config.rider_intent");
    }
    return out;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) decode arguments, the caller pins the roles
BootstrapState decode_bootstrap_state(const bike::io::Value &state,
                                      const mjModel &model) {
    const vread::Dict root{.value = &vread::mapping(vread::Node{&state},
                                                   "state"),
                           .path = "state"};
    constexpr auto required = vread::keys(
        "schema", "model_digest", "model_dims", "integration_state",
        "model_mutable", "tire", "drive", "rider_contacts",
        "rider_controller", "rider_intent", "signals", "runtime");
    const int version = vread::integer32(vread::field(root, "schema"), "state.schema");
    if (version == static_cast<int>(bike::spatial::kSpatialBootstrapSchema)) {
        vread::exact(root, required, vread::keys("spatial"));
        (void)vread::field(root, "spatial");
    } else vread::exact(root, required);
    if (version != kRuntimeSchema && version != static_cast<int>(bike::spatial::kSpatialBootstrapSchema))
        vread::invalid("state.schema", "unsupported bootstrap schema");

    BootstrapState out;
    out.schema = version;
    out.model_digest = expected_model_digest(state);
    out.model_dims = decode_model_dims(root, model);
    out.integration_state = decode_integration_state(root, model);
    out.model_mutable = decode_model_mutable(root, model);
    if (version == kRuntimeSchema) out.tire = decode_tire_state(root);
    else {
        out.spatial = decode_spatial_bootstrap(*vread::field(root, "spatial").value);
        const auto tire = vread::section(root, "tire");
        vread::exact(tire, vread::keys("pneumatic3d_state", "identity", "encoding", "payload"));
        if (vread::integer(vread::field(tire, "pneumatic3d_state"), tire.child("pneumatic3d_state")) != 1)
            vread::invalid(tire.child("pneumatic3d_state"), "unsupported tyre schema");
        if (vread::string(vread::field(tire, "encoding"), tire.child("encoding")) != "base64")
            vread::invalid(tire.child("encoding"), "expected canonical base64");
        const auto identity = bike::pneumatic3d::tyre_identity_from_wire(*vread::field(tire, "identity").value);
        if (identity.topology_fingerprint_hex != out.spatial->topology_fingerprint ||
            identity.terrain_sha256_hex != out.spatial->terrain_sha256 ||
            identity.calibration_sha256_hex != out.spatial->calibration_sha256)
            vread::invalid(tire.child("identity"), "differs from bootstrap spatial identity");
        const auto bytes = bike::pneumatic3d::base64_decode_strict(
            vread::string(vread::field(tire, "payload"), tire.child("payload")));
        if (!bytes) vread::invalid(tire.child("payload"), bytes.error());
        auto decoded = bike::pneumatic3d::decode_tyre_state(*bytes, identity);
        if (!decoded) vread::invalid(tire.child("payload"), decoded.error());
        out.tire3d = std::move(*decoded);
        // A disposable arena makes qpos offsets follow the engine's actual
        // integration layout. No live model/data are touched by validation.
        const std::unique_ptr<mjData, decltype(&mj_deleteData)> data(engine::make_data(&model), &mj_deleteData);
        if (!data) throw std::runtime_error("state.integration_state: cannot allocate validation arena");
        engine::set_state(&model, data.get(), out.integration_state.data(), mjSTATE_INTEGRATION);
        const auto types = std::views::counted(model.jnt_type, model.njnt);
        const auto addresses = std::views::counted(model.jnt_qposadr, model.njnt);
        const auto q = std::views::counted(data->qpos, model.nq);
        for (std::size_t j = 0; j < types.size(); ++j) {
            if (types[j] != mjJNT_FREE && types[j] != mjJNT_BALL) continue;
            const auto a = static_cast<std::size_t>(addresses[j] + (types[j] == mjJNT_FREE ? 3 : 0));
            if (a > q.size() || q.size() - a < 4)
                vread::invalid("state.integration_state.qpos", "quaternion span exceeds model");
            const double norm = std::hypot(std::hypot(q[a], q[a+1]), std::hypot(q[a+2], q[a+3]));
            if (std::abs(norm - 1.) > 1e-9)
                vread::invalid("state.integration_state.qpos", "non-unit free/ball quaternion");
        }
    }
    out.drive = drivetrain::decode_drive_snapshot(
        *vread::field(root, "drive").value);

    // state.rider_contacts — the document; the ctor decodes it against the
    // live writer topology (parse_rider_contacts_state's decode position).
    out.rider_contacts = *vread::field(root, "rider_contacts").value;

    // The staged step-state documents stay owned trees — they decode at
    // the physical-step/step-state positions inside the ctor.
    out.rider_controller = owned_optional(root, "rider_controller");
    out.rider_intent = owned_mapping(root, "rider_intent");
    out.signals = owned_mapping(root, "signals");

    const vread::Dict runtime_section = vread::section(root, "runtime");
    const auto runtime_keys=vread::keys("step", "generation", "measurement_decimation",
                             "applied_control", "held_control",
                             "held_rider_terms", "rollback_hold",
                             "research_accounting_valid", "initializing",
                             "filters", "balance", "crash", "contacts",
                             "contact_query", "probe_query", "energy",
                             "history", "model_status", "monitor");
    if(version==2)vread::exact(runtime_section,runtime_keys,vread::keys("continuation"));
    else vread::exact(runtime_section,runtime_keys);
    out.step = vread::integer(vread::field(runtime_section, "step"),
                              runtime_section.child("step"));
    if (out.step < 0)
        vread::invalid(runtime_section.child("step"),
                       "expected nonnegative integer");
    out.generation = vread::integer32(
        vread::field(runtime_section, "generation"),
        runtime_section.child("generation"));
    if (out.generation <= 0)
        vread::invalid(runtime_section.child("generation"),
                       "expected positive integer");
    out.runtime = bike::io::Value{bio::Value::Object(*runtime_section.value)};
    return out;
}

void verify_model_dims(const ModelDims &dims, const mjModel &model) {
    // The runtime ctor's re-verification of the decoded inventory against
    // the live model — the same per-field comparison decode_model_dims
    // ran, so a fabricated bootstrap cannot slip a mismatched record past
    // ownership.
    for (const auto &[name, pair] : dims_fields(model))
        if (dims.*(pair.second) != pair.first) {
            const std::string key(name);
            throw std::invalid_argument("state.model_dims." + key +
                                        ": model dimension mismatch");
        }
}

SpatialBootstrapState decode_spatial_bootstrap(const bio::Value& value) {
    const vread::Dict d{.value=&vread::mapping({&value}, "state.spatial"), .path="state.spatial"};
    vread::exact(d, vread::keys("spatial_contract", "root_kind", "topology_fingerprint",
        "terrain_sha256", "calibration_sha256", "bootstrap_phase", "hold", "brake", "steer", "outcome"));
    for (std::size_t i=0; i<d.size(); ++i)
        for (std::size_t j=0; j<i; ++j)
            if ((*d.value)[i].first==(*d.value)[j].first)
                vread::invalid(d.child((*d.value)[i].first), "duplicate key");
    if (vread::integer(vread::field(d,"spatial_contract"),d.child("spatial_contract")) != bike::spatial::kSpatialContract)
        vread::invalid(d.child("spatial_contract"),"unsupported contract");
    SpatialBootstrapState out;
    const auto root = bike::spatial::parse_root_kind(vread::string(vread::field(d,"root_kind"),d.child("root_kind")));
    if (!root || *root==bike::spatial::RootKind::planar3)
        vread::invalid(d.child("root_kind"), "expected constrained or free root");
    out.root_kind=*root;
    const auto digest = [&](const char* name, bool allow_empty=false) {
        const auto& s=vread::string(vread::field(d,name),d.child(name));
        if ((!allow_empty || !s.empty()) && (s.size()!=64 || s.find_first_not_of("0123456789abcdef")!=std::string::npos))
            vread::invalid(d.child(name), "expected lowercase SHA-256");
        return s;
    };
    out.topology_fingerprint=digest("topology_fingerprint");
    out.terrain_sha256=digest("terrain_sha256");
    out.calibration_sha256=digest("calibration_sha256",true);
    const auto phase=bike::spatial::parse_phase(vread::string(vread::field(d,"bootstrap_phase"),d.child("bootstrap_phase")));
    if(!phase)vread::invalid(d.child("bootstrap_phase"),"unknown phase");
    out.bootstrap_phase=*phase;
    try {
        out.hold=bike::spatial::decode_hold(*vread::field(d,"hold").value);
        out.brake=bike::spatial::decode_brake(*vread::field(d,"brake").value);
        out.steer=bike::spatial::decode_steer(*vread::field(d,"steer").value);
        out.outcome=bike::spatial::decode_outcome(*vread::field(d,"outcome").value);
    } catch(const std::invalid_argument& e) { vread::invalid("state.spatial",e.what()); }
    if(out.hold.phase!=out.bootstrap_phase)
        vread::invalid(d.child("hold.phase"),"differs from bootstrap_phase");
    return out;
}

bio::Value encode_spatial_bootstrap(const SpatialBootstrapState& s) {
    using namespace setupvalue;
    Value value{Object{{"spatial_contract",integer(bike::spatial::kSpatialContract)},
        {"root_kind",text(bike::spatial::to_string(s.root_kind))},
        {"topology_fingerprint",text(s.topology_fingerprint)},
        {"terrain_sha256",text(s.terrain_sha256)}, {"calibration_sha256",text(s.calibration_sha256)},
        {"bootstrap_phase",text(bike::spatial::to_string(s.bootstrap_phase))},
        {"hold",bike::spatial::encode_hold(s.hold)}, {"brake",bike::spatial::encode_brake(s.brake)},
        {"steer",bike::spatial::encode_steer(s.steer)}, {"outcome",bike::spatial::encode_outcome(s.outcome)}}};
    (void)decode_spatial_bootstrap(value);
    return value;
}

void validate_schema_family(const RuntimeConfig& config, int state_schema) {
    const auto& writer=bio::as_object(config.writer_config,"config.writer_config");
    const auto* schema=bio::find_member(writer,"schema");
    if(!schema)vread::invalid("config.writer_config.schema","missing required key");
    const auto version=bio::as_int64(*schema,"config.writer_config.schema");
    if(config.schema==kRuntimeSchema && !config.spatial && state_schema==kRuntimeSchema &&
       (version==1||version==2))return;
    if(config.schema!=static_cast<int>(bike::spatial::kSpatialRuntimeSchema) ||
       state_schema!=static_cast<int>(bike::spatial::kSpatialBootstrapSchema) ||
       version!=bike::spatial::kSpatialWriterSchema || !config.spatial ||
       config.spatial->mode!=bike::spatial::Mode::spatial3d)
        vread::invalid("bootstrap.schema_family","expected planar (1,1|2,1) or spatial (2,3,2)");
    nativecfg::validate_spatial_writer_envelope(config.writer_config);
    for(auto name:{"drive","suspension","resistance","tire"})
        if(!bio::find_member(writer,name))vread::invalid("config.writer_config."+std::string(name),"required spatial producer");
}

void validate_spatial_identity(const RuntimeConfig& config, const SpatialBootstrapState& state,
                               const bike::spatial::Topology& topology) {
    if(!config.spatial)vread::invalid("config.spatial","required");
    if(config.spatial->root!=state.root_kind || topology.root.kind!=state.root_kind)
        vread::invalid("state.spatial.root_kind","differs from configured model topology");
    if(topology.fingerprint_hex!=state.topology_fingerprint)
        vread::invalid("state.spatial.topology_fingerprint","differs from rebound model");
    if(!config.terrain3d || config.terrain3d->fingerprint_hex()!=state.terrain_sha256)
        vread::invalid("state.spatial.terrain_sha256","differs from owned physical asset");
    if(state.calibration_sha256!=config.spatial->tyre.lateral_coefficients_sha256)
        vread::invalid("state.spatial.calibration_sha256","differs from configured profile");
    const vread::Dict root{.value=&bio::as_object(config.writer_config,"config.writer_config"),.path="config.writer_config"};
    const auto writer=vread::section(root,"spatial");
    for(const auto& [key,expected]:std::array<std::pair<const char*,std::string>,3>{{
        {"root_kind",std::string(bike::spatial::to_string(state.root_kind))},
        {"topology_fingerprint",state.topology_fingerprint},{"terrain_sha256",state.terrain_sha256}}})
        if(vread::string(vread::field(writer,key),writer.child(key))!=expected)
            vread::invalid(writer.child(key),"differs from bootstrap/model identity");
}

} // namespace runtime
