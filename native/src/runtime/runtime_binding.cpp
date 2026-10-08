// runtime_binding.cpp — NativeRideRuntime: the owned C++ ride runtime.
//
// Construction order (every failure before Stepper construction leaves
// nothing behind; a later failure unwinds the unique_ptr):
//   1. config envelope validation (runtime_schema=1, all required fields)
//   2. artifact identity — sha256(model.mjb) == state.model_digest
//   3. Stepper construction (schema-2 writer_config parse happens inside)
//   4. bootstrap decode — model_dims/digest, integration width, resolved
//      model coefficients, mechanical snapshots, staged controller state
//   5. one ordered restore: reset_data -> model coefficients -> set_const
//      -> integration vector -> writer snapshots -> runtime counters
//      (mj_setConst rewrites qpos, so the integration vector lands after it)
//
// The integration vector is restored verbatim — mj_getState/mj_setState
// round-trip qacc_warmstart exactly, so no forward() runs here; derived
// mjData fields stay at reset values until the first forward (the same
// deferral Stepper::set_state documents).
#include "runtime_binding.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#include <mujoco/mujoco.h>
#include <nanobind/stl/string.h>

#include "../binding_arrays.hpp"
#include "../binding_readers.hpp"
#include "../engine_call.hpp"
#include "../stepper.hpp"

namespace nb = nanobind;

namespace {

// ---- FIPS 180-4 SHA-256 ------------------------------------------------
// The project links no crypto library; this compact implementation covers
// the single digest the bootstrap verifies (model artifact identity).

class Sha256 {
public:
    void update(std::span<const std::byte> bytes) noexcept {
        message_bytes_ += bytes.size();
        feed(bytes);
    }

    [[nodiscard]] std::string hex() {
        const std::uint64_t bit_length = message_bytes_ * 8U;
        const std::byte start{0x80};
        feed(std::span<const std::byte>{&start, 1});
        const std::byte zero{0x00};
        while (block_used_ != 56U)
            feed(std::span<const std::byte>{&zero, 1});
        std::array<std::byte, 8> length_bytes{};
        for (std::size_t i = 0; i < 8; ++i)
            length_bytes[7U - i] =
                static_cast<std::byte>(static_cast<unsigned char>(
                    (bit_length >> (i * 8U)) & 0xffU));
        feed(length_bytes);
        static constexpr std::array<char, 16> digits{'0', '1', '2', '3', '4',
            '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
        std::string out;
        out.reserve(64);
        for (const std::uint32_t word : h_)
            for (const int shift : {28, 24, 20, 16, 12, 8, 4, 0})
                out.push_back(digits[static_cast<std::size_t>(
                    (word >> static_cast<unsigned>(shift)) & 0xfU)]);
        return out;
    }

private:
    void feed(std::span<const std::byte> bytes) noexcept {
        while (!bytes.empty()) {
            const std::size_t take =
                std::min(bytes.size(), block_.size() - block_used_);
            std::ranges::copy(bytes.first(take),
                              std::span(block_).subspan(block_used_).begin());
            block_used_ += take;
            bytes = bytes.subspan(take);
            if (block_used_ == block_.size()) {
                compress();
                block_used_ = 0;
            }
        }
    }

    void compress() noexcept {
        std::array<std::uint32_t, 64> w{};
        for (std::size_t i = 0; i < 16; ++i)
            w[i] = std::to_integer<std::uint32_t>(block_[i * 4]) << 24U |
                   std::to_integer<std::uint32_t>(block_[i * 4 + 1]) << 16U |
                   std::to_integer<std::uint32_t>(block_[i * 4 + 2]) << 8U |
                   std::to_integer<std::uint32_t>(block_[i * 4 + 3]);
        for (std::size_t i = 16; i < 64; ++i)
            w[i] = w[i - 16] +
                   (std::rotr(w[i - 15], 7) ^ std::rotr(w[i - 15], 18) ^
                    (w[i - 15] >> 3U)) +
                   w[i - 7] +
                   (std::rotr(w[i - 2], 17) ^ std::rotr(w[i - 2], 19) ^
                    (w[i - 2] >> 10U));
        std::array<std::uint32_t, 8> v = h_;
        for (std::size_t i = 0; i < 64; ++i) {
            const std::uint32_t s1 = std::rotr(v[4], 6) ^
                                     std::rotr(v[4], 11) ^
                                     std::rotr(v[4], 25);
            const std::uint32_t ch =
                (v[4] & v[5]) ^ (~v[4] & v[6]);
            const std::uint32_t t1 =
                v[7] + s1 + ch + kRound[i] + w[i];
            const std::uint32_t s0 = std::rotr(v[0], 2) ^
                                     std::rotr(v[0], 13) ^
                                     std::rotr(v[0], 22);
            const std::uint32_t maj =
                (v[0] & v[1]) ^ (v[0] & v[2]) ^
                (v[1] & v[2]);
            const std::uint32_t t2 = s0 + maj;
            v[7] = v[6];
            v[6] = v[5];
            v[5] = v[4];
            v[4] = v[3] + t1;  // e <- d + t1, before d shifts out
            v[3] = v[2];
            v[2] = v[1];
            v[1] = v[0];
            v[0] = t1 + t2;
        }
        for (std::size_t i = 0; i < 8; ++i) h_[i] += v[i];
    }

    static constexpr std::array<std::uint32_t, 64> kRound{
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
        0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
        0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
        0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
        0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
        0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
        0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
        0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
        0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
        0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
        0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
        0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};
    std::array<std::uint32_t, 8> h_{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                    0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                    0x1f83d9abU, 0x5be0cd19U};
    std::array<std::byte, 64> block_{};
    std::size_t block_used_ = 0;
    std::uint64_t message_bytes_ = 0;
};

[[nodiscard]] std::string sha256_file(const std::string &path) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
        throw std::invalid_argument("model_path: cannot read artifact file");
    Sha256 hash;
    // Heap buffer: a stack copy would dominate the caller's frame when this
    // one-shot reader gets inlined.
    const auto buffer = std::make_unique<std::array<char, 16384>>();
    const std::span<char> view(*buffer);
    while (file) {
        file.read(view.data(), static_cast<std::streamsize>(view.size()));
        const auto got = static_cast<std::size_t>(file.gcount());
        hash.update(std::as_bytes(view).first(got));
    }
    return hash.hex();
}

[[nodiscard]] std::string artifact_path(nb::handle value) {
    nb::object resolved;
    if (nb::isinstance<nb::str>(value)) {
        resolved = nb::borrow<nb::object>(value);
    } else {
        resolved = nb::module_::import_("os").attr("fspath")(value);
        if (!nb::isinstance<nb::str>(resolved))
            throw std::invalid_argument(
                "model_path: expected a str or os.PathLike");
    }
    return wire::string(resolved, "model_path");
}

// ---- shared field readers ----------------------------------------------

[[nodiscard]] double positive_field(const wire::Dict &d, const char *key) {
    const double value =
        wire::finite_real(wire::field(d, key), d.child(key));
    if (!(value > 0.)) wire::invalid(d.child(key), "expected positive value");
    return value;
}

[[nodiscard]] double nonnegative_field(const wire::Dict &d, const char *key) {
    const double value =
        wire::finite_real(wire::field(d, key), d.child(key));
    if (value < 0.)
        wire::invalid(d.child(key), "expected nonnegative value");
    return value;
}

[[nodiscard]] int positive_int_field(const wire::Dict &d, const char *key) {
    const int value = wire::integer32(wire::field(d, key), d.child(key));
    if (value <= 0)
        wire::invalid(d.child(key), "expected positive integer");
    return value;
}

[[nodiscard]] runtime::JointReference joint_reference(const wire::Dict &d,
                                                      const char *key) {
    const wire::Dict ref = wire::section(d, key);
    wire::exact(ref, wire::keys("joint", "dof_index"));
    runtime::JointReference out{
        .joint = wire::string(wire::field(ref, "joint"), ref.child("joint")),
        .dof_index = wire::integer32(wire::field(ref, "dof_index"),
                                     ref.child("dof_index"))};
    if (out.dof_index < 0)
        wire::invalid(ref.child("dof_index"), "expected nonnegative index");
    return out;
}

[[nodiscard]] std::optional<std::string> optional_string_field(
    const wire::Dict &d, const char *key) {
    const nb::object value = wire::field(d, key);
    if (value.is_none()) return std::nullopt;
    return wire::string(value, d.child(key));
}

// owned dict-copy of a required mapping field.
[[nodiscard]] nb::object owned_mapping(const wire::Dict &d, const char *key) {
    const nb::dict source =
        wire::mapping(wire::field(d, key), d.child(key));
    return runtime::owned_copy(source, d.child(key));
}

[[nodiscard]] nb::object owned_optional(const wire::Dict &d, const char *key) {
    const nb::object value = wire::field(d, key);
    if (value.is_none()) return nb::borrow<nb::object>(value);
    return runtime::owned_copy(
        wire::mapping(value, d.child(key)), d.child(key));
}

} // namespace

namespace runtime {

// NOLINTNEXTLINE(misc-no-recursion) depth mirrors the plain-data tree
nb::object owned_copy(nb::handle value, std::string_view path) {
    if (value.is_none() || nb::isinstance<nb::str>(value) ||
        nb::isinstance<nb::bool_>(value) ||
        wire::numeric_boolean(value))
        return nb::borrow<nb::object>(value);
    const auto numbers = nb::module_::import_("numbers");
    if (nb::isinstance(value, numbers.attr("Integral")) ||
        nb::isinstance(value, numbers.attr("Real")))
        return nb::borrow<nb::object>(value);
    if (nb::isinstance<nb::dict>(value)) {
        nb::dict out;
        for (const auto item : nb::borrow<nb::dict>(value)) {
            if (!nb::isinstance<nb::str>(item.first))
                wire::invalid(path, "expected string keys");
            out[nb::handle(item.first)] = owned_copy(item.second, path);
        }
        return out;
    }
    if (nb::isinstance<nb::list>(value)) {
        nb::list out;
        for (nb::handle const item : nb::borrow<nb::list>(value))
            out.append(owned_copy(item, path));
        return out;
    }
    if (nb::isinstance<nb::tuple>(value)) {
        nb::list stage;
        for (nb::handle const item : nb::borrow<nb::tuple>(value))
            stage.append(owned_copy(item, path));
        return nb::steal<nb::object>(PySequence_Tuple(stage.ptr()));
    }
    if (nb::isinstance(value, nb::module_::import_("numpy").attr("ndarray"))) {
        const auto kind =
            wire::string(value.attr("dtype").attr("kind"), path);
        if (kind != "b" && kind != "i" && kind != "u" && kind != "f" &&
            kind != "c")
            wire::invalid(path, "expected a numeric array dtype");
        return value.attr("copy")();
    }
    wire::invalid(path, "expected plain data (dict/list/tuple/scalar/array)");
}

RuntimeConfig parse_runtime_config(nb::handle raw) {
    const wire::Dict root{.value = wire::mapping(raw, "config"),
                          .path = "config"};
    constexpr auto required = wire::keys(
        "runtime_schema", "timestep_s", "control_period_s",
        "control_period_steps", "physics_mode", "drive_mode", "strict",
        "record_decimation", "writer_config", "rider_controller",
        "rider_intent", "monitors", "geometry");
    wire::exact(root, required);
    if (wire::integer32(wire::field(root, "runtime_schema"),
                        root.child("runtime_schema")) != kRuntimeSchema)
        wire::invalid(root.child("runtime_schema"),
                      "unsupported runtime schema");

    RuntimeConfig out;
    out.timestep_s = positive_field(root, "timestep_s");
    out.control_period_s = positive_field(root, "control_period_s");
    out.control_period_steps =
        positive_int_field(root, "control_period_steps");
    if (std::abs(out.timestep_s * out.control_period_steps -
                 out.control_period_s) > 1e-9 * out.control_period_s)
        wire::invalid(root.child("control_period_s"),
                      "control_period_steps * timestep_s mismatch");
    if (wire::string(wire::field(root, "physics_mode"),
                     root.child("physics_mode")) != "physical")
        wire::invalid(root.child("physics_mode"), "unsupported physics mode");
    if (wire::string(wire::field(root, "drive_mode"),
                     root.child("drive_mode")) != "articulated_effort")
        wire::invalid(root.child("drive_mode"), "unsupported drive mode");
    out.strict =
        wire::boolean(wire::field(root, "strict"), root.child("strict"));
    out.record_decimation = positive_int_field(root, "record_decimation");

    out.writer_config = owned_mapping(root, "writer_config");
    out.rider_controller = owned_mapping(root, "rider_controller");
    out.rider_intent = owned_mapping(root, "rider_intent");

    const wire::Dict monitors = wire::section(root, "monitors");
    wire::exact(monitors,
                wire::keys("balance_floor_mps", "balance_dwell_s",
                           "balance_grace_s", "grounded_hold_s"));
    out.monitors = MonitorConfig{
        .balance_floor_mps = positive_field(monitors, "balance_floor_mps"),
        .balance_dwell_s = positive_field(monitors, "balance_dwell_s"),
        .balance_grace_s = nonnegative_field(monitors, "balance_grace_s"),
        .grounded_hold_s = nonnegative_field(monitors, "grounded_hold_s")};

    const wire::Dict geometry = wire::section(root, "geometry");
    wire::exact(geometry,
                wire::keys("crank_length_m", "brake_dofs", "wheel_bodies",
                           "com_marker_site", "pose"));
    out.geometry.crank_length_m = positive_field(geometry, "crank_length_m");
    const wire::Dict brake = wire::section(geometry, "brake_dofs");
    wire::exact(brake, wire::keys("front", "rear"));
    out.geometry.front_brake = joint_reference(brake, "front");
    out.geometry.rear_brake = joint_reference(brake, "rear");
    const wire::Dict bodies = wire::section(geometry, "wheel_bodies");
    wire::exact(bodies, {}, wire::keys("front", "rear"));
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
    return out;
}

std::string expected_model_digest(nb::handle raw_state) {
    const wire::Dict root{.value = wire::mapping(raw_state, "state"),
                          .path = "state"};
    return wire::string(wire::field(root, "model_digest"),
                        root.child("model_digest"));
}

// ---- bootstrap section parsers ------------------------------------------
// Each decoder owns its wire::Dict temporaries; keeping them in separate
// functions holds every frame under the 8 KiB guard.
namespace {

void parse_model_dims(const wire::Dict &root, const mjModel &model) {
    const wire::Dict dims = wire::section(root, "model_dims");
    wire::exact(dims,
                wire::keys("nq", "nv", "nu", "na", "nbody", "njnt", "ntendon",
                           "nwrap", "nsite", "ngeom", "neq", "nsensor",
                           "nsensordata"));
    const std::array<std::pair<std::string_view, mjtSize>, 13> actual_dims{{
        {"nq", model.nq},         {"nv", model.nv},
        {"nu", model.nu},         {"na", model.na},
        {"nbody", model.nbody},   {"njnt", model.njnt},
        {"ntendon", model.ntendon}, {"nwrap", model.nwrap},
        {"nsite", model.nsite},   {"ngeom", model.ngeom},
        {"neq", model.neq},       {"nsensor", model.nsensor},
        {"nsensordata", model.nsensordata},
    }};
    for (const auto &[name, actual] : actual_dims) {
        const std::string key(name);
        const auto expected = wire::integer(
            wire::field(dims, key.c_str()), dims.child(key.c_str()));
        if (!std::cmp_equal(expected, actual))
            wire::invalid(dims.child(key.c_str()),
                          "model dimension mismatch");
    }
}

std::vector<double> parse_integration_state(const wire::Dict &root,
                                            const mjModel &model) {
    std::vector<double> values = wire::vector(
        wire::field(root, "integration_state"), "state.integration_state");
    const mjtSize width =
        engine::state_size(&model, static_cast<int>(mjSTATE_INTEGRATION));
    if (!std::cmp_equal(values.size(), width))
        wire::invalid("state.integration_state",
                      "width does not match the model");
    return values;
}

ModelMutableState parse_model_mutable(const wire::Dict &root,
                                      const mjModel &model) {
    ModelMutableState out;
    const wire::Dict mutable_section = wire::section(root, "model_mutable");
    wire::exact(mutable_section, wire::keys("dof_frictionloss", "site_pos"));
    const wire::Dict frictionloss =
        wire::section(mutable_section, "dof_frictionloss");
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
    for (const auto item : frictionloss.value) {
        const std::string side =
            wire::string(item.first, frictionloss.path);
        const wire::Dict entry{
            .value = wire::mapping(item.second, frictionloss.child(side.c_str())),
            .path = frictionloss.child(side.c_str())};
        wire::exact(entry, wire::keys("joint", "dof_index", "value"));
        FrictionLossEntry row{
            .joint = wire::string(wire::field(entry, "joint"),
                                  entry.child("joint")),
            .dof_index = wire::integer32(wire::field(entry, "dof_index"),
                                         entry.child("dof_index")),
            .dof_adr = 0,
            .value = wire::finite_real(wire::field(entry, "value"),
                                       entry.child("value"))};
        const int joint =
            mj_name2id(&model, mjOBJ_JOINT, row.joint.c_str());
        if (joint < 0)
            wire::invalid(entry.child("joint"), "unknown joint");
        if (row.dof_index < 0 ||
            row.dof_index >= dof_count(joint))
            wire::invalid(entry.child("dof_index"), "out of range");
        row.dof_adr = dofadr[static_cast<std::size_t>(joint)] +
                      row.dof_index;
        out.dof_frictionloss.push_back(std::move(row));
    }
    const wire::Dict site_pos = wire::section(mutable_section, "site_pos");
    for (const auto item : site_pos.value) {
        const std::string name = wire::string(item.first, site_pos.path);
        const int site = mj_name2id(&model, mjOBJ_SITE, name.c_str());
        if (site < 0)
            wire::invalid(site_pos.child(name.c_str()), "unknown site");
        out.site_pos.push_back(SitePosEntry{
            .site = name, .site_id = site,
            .value = wire::fixed<3>(item.second, site_pos.child(name.c_str()))});
    }
    return out;
}

TireStateRow parse_tire_state(const wire::Dict &root) {
    TireStateRow out;
    const wire::Dict tire = wire::section(root, "tire");
    wire::exact(tire, wire::keys("names", "row"));
    for (nb::handle const value :
         wire::sequence(wire::field(tire, "names"), tire.child("names")))
        out.names.push_back(wire::string(value, tire.child("names")));
    // NaN marks unset brush columns — real, not finite_real.
    for (nb::handle const value :
         wire::sequence(wire::field(tire, "row"), tire.child("row")))
        out.row.push_back(wire::real(value, tire.child("row")));
    if (out.names.size() != out.row.size())
        wire::invalid(tire.child("row"), "names/row width mismatch");
    return out;
}

} // namespace

BootstrapState parse_bootstrap(nb::handle raw_state, const mjModel &model,
                               const Stepper &stepper) {
    const wire::Dict root{.value = wire::mapping(raw_state, "state"),
                          .path = "state"};
    constexpr auto required = wire::keys(
        "schema", "model_digest", "model_dims", "integration_state",
        "model_mutable", "tire", "drive", "rider_contacts",
        "rider_controller", "rider_intent", "signals", "runtime");
    wire::exact(root, required);
    if (wire::integer32(wire::field(root, "schema"), "state.schema") !=
        kRuntimeSchema)
        wire::invalid("state.schema", "unsupported bootstrap schema");

    BootstrapState out;
    out.model_digest = expected_model_digest(raw_state);
    parse_model_dims(root, model);
    out.integration_state = parse_integration_state(root, model);
    out.model_mutable = parse_model_mutable(root, model);
    out.tire = parse_tire_state(root);
    out.drive = parse_drive_snapshot(wire::field(root, "drive"),
                                     "state.drive");

    const nb::object contacts = wire::field(root, "rider_contacts");
    if (!contacts.is_none()) {
        try {
            out.rider_contacts =
                parse_rider_contacts_state(contacts, stepper.rider_contacts());
        } catch (const std::logic_error &) {
            wire::invalid("state.rider_contacts",
                          "the runtime config has no rider_contacts writer");
        }
    }

    out.rider_controller = owned_optional(root, "rider_controller");
    out.rider_intent = owned_mapping(root, "rider_intent");
    out.signals = owned_mapping(root, "signals");

    const wire::Dict runtime_section = wire::section(root, "runtime");
    wire::exact(runtime_section,
                wire::keys("step", "generation", "record_decimation",
                           "applied_control", "held_control",
                           "held_rider_terms", "rollback_hold",
                           "research_accounting_valid", "initializing",
                           "filters", "balance", "crash", "contacts",
                           "contact_query", "probe_query", "energy", "history",
                           "model_status", "monitor"));
    out.step = wire::integer(wire::field(runtime_section, "step"),
                             runtime_section.child("step"));
    if (out.step < 0)
        wire::invalid(runtime_section.child("step"),
                      "expected nonnegative integer");
    out.generation = wire::integer32(
        wire::field(runtime_section, "generation"),
        runtime_section.child("generation"));
    if (out.generation <= 0)
        wire::invalid(runtime_section.child("generation"),
                      "expected positive integer");
    out.runtime = runtime::owned_copy(runtime_section.value,
                                      runtime_section.path);
    return out;
}

// ---- NativeRideRuntime --------------------------------------------------

void NativeRideRuntime::require_open() const {
    if (closed_) throw std::logic_error("NativeRideRuntime is closed");
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters) positional signature is the spec'd Python API
NativeRideRuntime::NativeRideRuntime(nb::handle model_path,
                                     nb::handle config, nb::handle state)
    : config_(parse_runtime_config(config)) {
    const std::string path = artifact_path(model_path);
    if (sha256_file(path) != expected_model_digest(state))
        throw std::invalid_argument(
            "state.model_digest: model artifact bytes do not match");
    stepper_ = std::make_unique<Stepper>(path, config_.writer_config);
    bootstrap_ = parse_bootstrap(state, *stepper_->model(), *stepper_);
    apply_bootstrap();
    generation_ = bootstrap_.generation;
}

void NativeRideRuntime::apply_bootstrap() {
    mjModel *const model = stepper_->model();
    mjData *const data = stepper_->data();
    stepper_->mutate([&] {
        engine::reset_data(model, data);
        const std::span<double> frictionloss =
            std::views::counted(model->dof_frictionloss, model->nv);
        for (const FrictionLossEntry &e :
             bootstrap_.model_mutable.dof_frictionloss)
            frictionloss[static_cast<std::size_t>(e.dof_adr)] = e.value;
        const std::span<double> site_pos =
            std::views::counted(model->site_pos, model->nsite * 3);
        for (const SitePosEntry &e : bootstrap_.model_mutable.site_pos)
            for (std::size_t i = 0; i < 3; ++i)
                site_pos[static_cast<std::size_t>(e.site_id) * 3U + i] =
                    e.value.at(i);
        // mj_setConst derives rest-state constants from qpos0 — it rewrites
        // qpos — so the captured integration vector restores strictly after
        // it, and the writer snapshots (which never touch live mjData) can
        // sit on either side.
        engine::set_const(model, data);
        engine::set_state(model, data, bootstrap_.integration_state.data(),
                          static_cast<int>(mjSTATE_INTEGRATION));
        stepper_->drive().restore(bootstrap_.drive);
        if (bootstrap_.rider_contacts.has_value()) {
            // assign_state moves — hand it a copy so the stored bootstrap
            // stays replayable across reset().
            auto [contact_state, probe] = *bootstrap_.rider_contacts;
            stepper_->rider_contacts().assign_state(std::move(contact_state),
                                                    std::move(probe));
        }
        stepper_->set_tire_state(bootstrap_.tire.names,
                                 bootstrap_.tire.row);
    });
    step_ = bootstrap_.step;
}

RuntimeSnapshot NativeRideRuntime::snapshot() const {
    require_open();
    const mjModel *const model = stepper_->model();
    const mjData *const data = stepper_->data();
    const mjtSize width =
        engine::state_size(model, static_cast<int>(mjSTATE_INTEGRATION));
    std::vector<double> values(static_cast<std::size_t>(width));
    engine::get_state(model, data, values.data(),
                      static_cast<int>(mjSTATE_INTEGRATION));
    return {.generation = generation_,
            .step = step_,
            .time_s = data->time,
            .integration_state = std::move(values)};
}

void NativeRideRuntime::reset() {
    require_open();
    apply_bootstrap();
    // PhysicalRuntime.reset() bumps generation; the stored bootstrap's
    // capture value is only the constructor's.
    ++generation_;
}

void NativeRideRuntime::close() {
    stepper_.reset();
    closed_ = true;
}

NativeRideRuntime::~NativeRideRuntime() = default;

void bind_runtime(const nb::module_ &module) {
    nb::class_<RuntimeSnapshot>(module, "RuntimeSnapshot")
        .def_ro("generation", &RuntimeSnapshot::generation)
        .def_ro("step", &RuntimeSnapshot::step)
        .def_ro("time_s", &RuntimeSnapshot::time_s)
        // Every access mints a fresh capsule-owned array — the stored
        // vector stays an internal copy the caller can never reach.
        .def_prop_ro("integration_state",
                     [](const RuntimeSnapshot &s) {
                         return wire::owned_array<double>(
                             s.integration_state);
                     },
                     nb::rv_policy::move);
    nb::class_<NativeRideRuntime>(module, "NativeRideRuntime")
        .def(nb::init<nb::handle, nb::handle, nb::handle>(),
             nb::arg("model_path"), nb::arg("config"), nb::arg("state"))
        .def("snapshot", &NativeRideRuntime::snapshot)
        .def("reset", &NativeRideRuntime::reset)
        .def("close", &NativeRideRuntime::close)
        .def_prop_ro("closed", &NativeRideRuntime::closed);
}

} // namespace runtime
