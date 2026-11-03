// Ports tests/reference/test_native_spindle_controller.py — the A2.3
// SpindleController parity family plus the F5 rear-leg-chain effort
// regression. The plant is the shared fixture the oracle's _plant() builds
// (tools/migration/_f5_spindle_capture.py): the pinned nine-actuator model
// (spindle pedals, pin saddle, connect grips) with the identical
// rider_controller wire config and pose. Every comparison is the golden
// node's recorded oracle output — f64 arrays close at the node's recorded
// rtol=1e-7/atol=1e-8, canonical-JSON state documents through the
// json_close deep comparison over bike::io::canonical_json(
// rider::encode_spindle_state()).
//
// F5 (test_spindle_rear_chain_effort_matches_python): the enabled-compute
// spindle path must emit nonzero rear hip/knee torques through the
// rear-driving crank half (crank_split.leg_shares assigns the stroke to
// the rear leg when cos(phase) < 0). The earlier adapter-only sweep left
// the runtime consumer untested; this freezes the oracle's own values.
#include "support/fixture_test.hpp"
#include "support/golden.hpp"
#include "diag.hpp"
#include "io/json.hpp"
#include "io/value.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/spindle_math.hpp"
#include "rider/state_codec.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <format>
#include <memory>
#include <numbers>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {
namespace bio = bike::io;

// Engine-buffer span adapters — the views::counted style step.cpp uses;
// raw ptr+size indexing is a hard error under -Wunsafe-buffer-usage.
std::span<const double> buf(const double *p, std::size_t n) {
    return std::views::counted(p, static_cast<std::ptrdiff_t>(n));
}
std::span<double> wbuf(double *p, std::size_t n) {
    return std::views::counted(p, static_cast<std::ptrdiff_t>(n));
}
std::span<const int> ibuf(const int *p, std::size_t n) {
    return std::views::counted(p, static_cast<std::ptrdiff_t>(n));
}

constexpr std::string_view kFile =
    "native_spindle_controller/";

std::string node_id(std::string_view symbol) {
    std::string key{symbol};
    if (key.starts_with("test_")) key.erase(0, 5);
    if (const auto bracket = key.find('['); bracket != std::string::npos) {
        auto param = std::string_view(key).substr(bracket + 1);
        if (param.ends_with(']')) param.remove_suffix(1);
        // param views key's tail: build the suffix before truncating key.
        const std::string suffix = "/" + bt::oracle::key(param);
        key.resize(bracket);
        key += suffix;
    }
    return std::string(kFile) + key;
}

struct Plant {
    mjModel *model = nullptr;
    mjData *data = nullptr;
    std::unique_ptr<rider::SpindleController> controller;
    Plant() = default;
    Plant(const Plant &) = delete;
    Plant &operator=(const Plant &) = delete;
    Plant(Plant &&other) noexcept
        : model(other.model), data(other.data),
          controller(std::move(other.controller)) {
        other.model = nullptr;
        other.data = nullptr;
    }
    Plant &operator=(Plant &&) = delete;
    ~Plant() {
        if (data != nullptr) mj_deleteData(data);
        if (model != nullptr) mj_deleteModel(model);
    }
};

const bio::Value &member(const bio::Value &value, const char *key) {
    const bio::Value *found =
        bio::find_member(bio::as_object(value, key), key);
    bt::require(found != nullptr,
                std::string("config.json is missing member ") + key);
    return *found;
}

// The _damped_config() fixture twin's surviving leg: rewrite the
// activation_tau_s lexeme inside the decoded rider_controller document
// (the TOML's 0.025 -> 0.05). The fixture's joint_passive_damping_nms_rad
// rewrite has no surviving schema key — spec Q13 removed passive damping.
void set_lexeme(bio::Value &value, std::string_view key, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                std::string_view lexeme) {
    auto &object = std::get<bio::Value::Object>(value.data);
    for (auto &[name, entry] : object)
        if (name == key) {
            entry.data = bio::Number{std::string(lexeme)};
            return;
        }
    throw std::runtime_error(std::string("config key not found: ") +
                             std::string(key));
}

void set_string(bio::Value &value, std::string_view key, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
                std::string_view text) {
    auto &object = std::get<bio::Value::Object>(value.data);
    for (auto &[name, entry] : object)
        if (name == key) {
            entry.data = std::string(text);
            return;
        }
    throw std::runtime_error(std::string("config key not found: ") +
                             std::string(key));
}

Plant make_plant(bool activated = false) {
    Plant plant;
    const auto mjb = bt::fixture("spindle/pinned/model.mjb");
    plant.model = mj_loadModel(mjb.c_str(), nullptr);
    bt::require(plant.model != nullptr, "spindle/pinned/model.mjb loads");
    plant.data = mj_makeData(plant.model);
    bt::require(plant.data != nullptr, "mj_makeData");
    bio::Value const doc =
        bio::parse_json(bt::text(bt::fixture("spindle/pinned/config.json")));
    bio::Value rider_doc = member(doc, "rider_controller");
    if (activated) set_lexeme(rider_doc, "activation_tau_s", "0.05");
    plant.controller = std::make_unique<rider::SpindleController>(
        plant.model,
        rider::decode_spindle_pose(member(doc, "pose"), "config.pose"),
        rider::decode_spindle_config(rider_doc),
        bio::number_double(
            bio::as_number(member(doc, "crank_length_m"),
                           "config.crank_length_m"),
            "config.crank_length_m"));
    return plant;
}

std::size_t joint_qposadr(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_JOINT, name);
    bt::require(id >= 0, std::string("joint missing: ") + name);
    return static_cast<std::size_t>(
        ibuf(m->jnt_qposadr,
             static_cast<std::size_t>(m->njnt))[static_cast<std::size_t>(id)]);
}

std::size_t joint_dofadr(const mjModel *m, const char *name) {
    const int id = mj_name2id(m, mjOBJ_JOINT, name);
    bt::require(id >= 0, std::string("joint missing: ") + name);
    return static_cast<std::size_t>(
        ibuf(m->jnt_dofadr,
             static_cast<std::size_t>(m->njnt))[static_cast<std::size_t>(id)]);
}

std::size_t nq(const mjModel *m) {
    return static_cast<std::size_t>(m->nq);
}
std::size_t nv(const mjModel *m) {
    return static_cast<std::size_t>(m->nv);
}
std::size_t nu(const mjModel *m) {
    return static_cast<std::size_t>(m->nu);
}

// _staged(): qpos/qvel writes then a full forward — the identical mjData
// the oracle controller reads.
void stage(Plant const &plant, double crank_q, double crank_rate, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
           double root_pitch = 0.,
           std::span<const std::pair<const char *, double>> rates = {}) {
  mj_resetData(plant.model, plant.data);
  wbuf(plant.data->qpos,
       nq(plant.model))[joint_qposadr(plant.model, "crank_spin")] = crank_q;
  wbuf(plant.data->qpos,
       nq(plant.model))[joint_qposadr(plant.model, "root_pitch")] = root_pitch;
  wbuf(plant.data->qvel,
       nv(plant.model))[joint_dofadr(plant.model, "crank_spin")] = crank_rate;
  for (const auto &[name, rate] : rates)
    wbuf(plant.data->qvel, nv(plant.model))[joint_dofadr(plant.model, name)] =
        rate;
  mj_forward(plant.model, plant.data);
}

std::vector<double> torque_vector(const rider::NamedEntries<double> &torques,
                                  const rider::SpindleController &ctrl) {
    std::vector<double> out;
    out.reserve(ctrl.joints().size());
    for (const auto &joint : ctrl.joints()) {
        const auto found =
            std::ranges::find_if(torques, [&](const auto &entry) {
                return entry.first == joint.name;
            });
        bt::require(found != torques.end(),
                    "compute returned every configured joint");
        out.push_back(found->second);
    }
    return out;
}

std::span<const std::byte> byte_span(const std::string &text) {
    return std::views::counted(
        reinterpret_cast<const std::byte *>(text.data()), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
        static_cast<std::ptrdiff_t>(text.size()));
}

// ArticulatedRiderController.state_dict() is the canonical 18-key document
// — _coast_hold_nm travels on the equilibrium material's flat
// state_control_coast_hold_nm channel, never this dict, so the
// spindle-authored member is dropped before the byte comparison.
std::string state_document(const rider::SpindleController &ctrl) {
    bio::Value encoded = rider::encode_spindle_state(ctrl.state());
    auto &object = std::get<bio::Value::Object>(encoded.data);
    std::erase_if(object, [](const auto &entry) {
        return entry.first == "coast_hold_nm";
    });
    return bio::canonical_json(encoded);
}

// Oracle-side numpy/libm math is not bitwise-stable between
// darwin-arm64 and linux-aarch64 (the F5 linux capture diverges ~2e-9
// through the leg-kinematics trig while darwin matches bit-for-bit), so
// f64 records carry kind='close' at rtol=1e-7/atol=1e-8 — three decades
// below any physically meaningful deviation. State documents embed the
// excitation solve's diagnostics on top of the held torques — the widest
// observed platform spread (solution_excitation_nm/5, 1.06e-8 relative)
// stays inside the same bound under the json_close deep comparison:
// structure (keys, order, array lengths, literal leaves) is exact;
// numeric leaves obey rtol=1e-7/atol=1e-8.
constexpr double kPlatformRtol = 1e-7, kPlatformAtol = 1e-8;

bool json_close(const bio::Value &actual, const bio::Value &expected, // NOLINT(misc-no-recursion) recursion mirrors the document tree
                std::string &path) {
    const auto *a_num = std::get_if<bio::Number>(&actual.data);
    const auto *e_num = std::get_if<bio::Number>(&expected.data);
    if (a_num != nullptr || e_num != nullptr) {
        if (a_num == nullptr || e_num == nullptr) {
            path += " (kind mismatch)";
            return false;
        }
        const double a = bio::number_double(*a_num, "state");
        const double e = bio::number_double(*e_num, "state");
        if (a == e ||
            (std::isfinite(a) && std::isfinite(e) &&
             std::abs(a - e) <= kPlatformAtol + kPlatformRtol * std::abs(e)))
            return true;
        path += std::format(" actual={} expected={}", a, e);
        return false;
    }
    if (const auto *a_obj = std::get_if<bio::Value::Object>(&actual.data)) {
        const auto *e_obj = std::get_if<bio::Value::Object>(&expected.data);
        if (e_obj == nullptr || a_obj->size() != e_obj->size()) {
            path += " (object shape)";
            return false;
        }
        for (std::size_t i = 0; i < a_obj->size(); ++i) {
            const auto &[a_key, a_val] = (*a_obj)[i];
            const auto &[e_key, e_val] = (*e_obj)[i];
            if (a_key != e_key) {
                path += std::format(" (key '{}' != '{}')", a_key, e_key);
                return false;
            }
            const auto parent = path.size();
            path += '/' + a_key;
            if (!json_close(a_val, e_val, path)) return false;
            path.resize(parent);
        }
        return true;
    }
    if (const auto *a_arr = std::get_if<bio::Value::Array>(&actual.data)) {
        const auto *e_arr = std::get_if<bio::Value::Array>(&expected.data);
        if (e_arr == nullptr || a_arr->size() != e_arr->size()) {
            path += " (array shape)";
            return false;
        }
        for (std::size_t i = 0; i < a_arr->size(); ++i) {
            const auto parent = path.size();
            path += std::format("/{}", i);
            if (!json_close((*a_arr)[i], (*e_arr)[i], path)) return false;
            path.resize(parent);
        }
        return true;
    }
    if (bio::canonical_json(actual) != bio::canonical_json(expected)) {
        path += " (literal mismatch)";
        return false;
    }
    return true;
}

// Legacy-wire normalization mirroring the decode side's accepted-but-
// ignored contract (R3): recordings captured before spec Q13 carry
// passive_damping_nm / solved_passive_nm / rider_passive_power_w /
// rider_passive_work_step_j members the current encoder no longer emits.
// Strip them from the EXPECTED document so the comparison stays strict —
// key set, order, array lengths and numeric leaves — on every surviving
// member; a regression elsewhere still fails structure- or value-first.
void strip_removed_passive_keys(bio::Value &value) { // NOLINT(misc-no-recursion) recursion mirrors the document tree
    if (auto *object = std::get_if<bio::Value::Object>(&value.data)) {
        std::erase_if(*object, [](const auto &entry) {
            return entry.first == "passive_damping_nm" ||
                   entry.first == "solved_passive_nm" ||
                   entry.first == "rider_passive_power_w" ||
                   entry.first == "rider_passive_work_step_j";
        });
        for (auto &[name, child] : *object)
            strip_removed_passive_keys(child);
    } else if (auto *array =
                   std::get_if<bio::Value::Array>(&value.data)) {
        for (auto &child : *array) strip_removed_passive_keys(child);
    }
}

void expect_state_close(bt::oracle::Node &node, const std::string &document,
                        std::string_view label) {
  const auto expected_bytes = node.bytes_input(label);
  const std::string_view source(
      reinterpret_cast<const char *>(expected_bytes.data()), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
      expected_bytes.size());
  bio::Value expected = bio::parse_json(source);
  strip_removed_passive_keys(expected);
  std::string path;
  bt::require(
      json_close(bio::parse_json(document), expected, path),
      std::string(label) + ": state diverges beyond platform tolerance at" +
          path);
}

// The recorded pair every compute-family node captures: the joint-ordered
// torque vector plus the full 18-key state_dict() canonical document.
void compare_compute(bt::oracle::Node &node,
                     const rider::NamedEntries<double> &torques,
                     const rider::SpindleController &ctrl,
                     std::string_view prefix = "compute") {
    const auto values = torque_vector(torques, ctrl);
    node.f64_close(values, std::string(prefix) + ".torques");
    expect_state_close(node, state_document(ctrl),
                       std::string(prefix) + ".state");
}

// ---- A2.3 pure kernels ----------------------------------------------------

void test_spindle_waveform_matches_python() {
    const std::pair<double, const char *> cases[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {0., "0.0"},
        {std::numbers::pi / 2., "1.5707963267948966"},
        {std::numbers::pi, "3.141592653589793"},
        {2. * std::numbers::pi - 1e-7, "6.283185207179586"},
    };
    for (const auto &[phase, id] : cases) {
        bt::oracle::Node node(node_id(
            std::format("test_spindle_waveform_matches_python[{}]", id)));
        const double actual =
            spindle::spindle_torque_waveform(60., phase, .5);
        const std::array<double, 1> scalar{actual};
        node.f64_close(scalar, "waveform");
        node.done();
    }
}

void test_effort_ceiling_matches_python() {
    const std::pair<double, const char *> cases[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {-3., "-3.0"}, {0., "0.0"}, {8., "8.0"}, {20., "20.0"},
    };
    for (const auto &[rate, id] : cases) {
        bt::oracle::Node node(node_id(
            std::format("test_effort_ceiling_matches_python[{}]", id)));
        const double actual = spindle::crank_effort_ceiling(250., 60., rate);
        const std::array<double, 1> scalar{actual};
        node.f64_close(scalar, "ceiling");
        node.done();
    }
}

bool throws_std(auto &&fn) {
    try {
        fn();
        return false;
    } catch (const std::exception &) {
        return true;
    }
}

void test_spindle_waveform_rejects_bad_inputs() {
    for (const auto [mean, phase, ripple] :
         {std::array{-1., 0., .5}, std::array{60., 0., 1.},
          std::array{60., std::nan(""), .5}})
        bt::require(throws_std([&] {
                        (void)spindle::spindle_torque_waveform(
                            mean, phase, ripple);
                    }),
                    "spindle_torque_waveform rejects bad input");
}

void test_effort_ceiling_rejects_bad_inputs() {
    for (const auto [power, limit, rate] :
         {std::array{-1., 60., 8.}, std::array{250., -60., 8.},
          std::array{250., 60., std::nan("")}})
        bt::require(throws_std([&] {
                        (void)spindle::crank_effort_ceiling(power, limit,
                                                            rate);
                    }),
                    "crank_effort_ceiling rejects bad input");
}

// ---- compute family --------------------------------------------------------

struct ComputeCase {
    double crank_q, crank_rate, effort;
    std::string node;
};

void compute_case(const ComputeCase &c) {
    Plant plant = make_plant();
    stage(plant, c.crank_q, c.crank_rate);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = c.effort;
    const auto torques =
        plant.controller->compute(plant.data, command, true, .00125, false);
    bt::oracle::Node node(c.node);
    compare_compute(node, torques, *plant.controller);
    node.done();
}

void test_spindle_compute_matches_python() {
    const std::pair<double, const char *> phases[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {0., "0.0"}, {.7, "0.7"},
        {std::numbers::pi / 2., "1.5707963267948966"}, {2.4, "2.4"},
    };
    const std::pair<double, const char *> efforts[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {0., "0.0"}, {40., "40.0"}, {120., "120.0"},
    };
    for (const auto &[effort, eid] : efforts)
        for (const auto &[crank_q, qid] : phases)
          compute_case(
              {.crank_q = crank_q,
               .crank_rate = 6.,
               .effort = effort,
               .node = node_id(std::format(
                   "test_spindle_compute_matches_python[{}-{}]", eid, qid))});
}

// F5: rear-leg-chain effort through the rear-driving crank half. The rear
// chain owns the stroke when cos(phase) < 0 (crank_split.leg_shares), and
// the hip/knee torques must be nonzero there — never collapse to ~0.
void test_spindle_rear_chain_effort_matches_python() {
    const std::pair<double, const char *> cases[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        {0., "0.0"},
        {std::numbers::pi / 2., "1.5707963267948966"},
        {std::numbers::pi, "3.141592653589793"},
        {3.67, "3.67"},
        {4.19, "4.19"},
    };
    for (const auto &[crank_q, id] : cases) { // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay) label buffer decays into snprintf/sizeof by design
        Plant plant = make_plant();
        stage(plant, crank_q, 6.);
        rider::RiderCommand command;
        command.mean_crank_torque_nm = 120.;
        const auto torques = plant.controller->compute(
            plant.data, command, true, .00125, false);
        const auto values = torque_vector(torques, *plant.controller);
        if (std::cos(crank_q) < 0.) {
            double hip = 0., knee = 0.;
            for (std::size_t i = 0; i < plant.controller->joints().size();
                 ++i) {
                const auto &name = plant.controller->joints()[i].name;
                if (name == "rider_hip_rear") hip = values[i];
                if (name == "rider_knee_rear") knee = values[i];
            }
            bt::require(std::abs(hip) > 1e-9 || std::abs(knee) > 1e-9,
                        std::format("crank_q={}: rear chain drives "
                                    "(hip {:.6f}, knee {:.6f})",
                                    crank_q, hip, knee));
        }
        bt::oracle::Node node(node_id(std::format(
            "test_spindle_rear_chain_effort_matches_python[{}]", id)));
        compare_compute(node, torques, *plant.controller);
        node.done();
    }
}

void test_spindle_disabled_command_matches_python() {
    Plant plant = make_plant();
    stage(plant, .3, 2.);
    rider::RiderCommand command;
    command.enabled = false;
    command.mean_crank_torque_nm = 80.;
    const auto torques =
        plant.controller->compute(plant.data, command, true, .00125, false);
    const auto values = torque_vector(torques, *plant.controller);
    bt::require(std::ranges::all_of(values,
                                    [](double v) { return v == 0.; }),
                "disabled command requests zero torque");
    bt::require(!plant.controller->state().support_diagnostics.stance_front &&
                    !plant.controller->state()
                         .support_diagnostics.stance_rear,
                "disabled stance flags are both false");
    bt::oracle::Node node(
        node_id("test_spindle_disabled_command_matches_python"));
    compare_compute(node, torques, *plant.controller);
    node.done();
}

void test_spindle_steady_state_bypasses_activation() {
    Plant plant = make_plant();
    stage(plant, 1.1, 7.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 90.;
    const auto torques =
        plant.controller->compute(plant.data, command, true, .00125, true);
    bt::oracle::Node node(
        node_id("test_spindle_steady_state_bypasses_activation"));
    compare_compute(node, torques, *plant.controller);
    node.done();
}

void test_spindle_coasting_command_matches_python() {
    Plant plant = make_plant();
    stage(plant, .35, 4.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 0.;
    command.crank_target_phase_rad = 1.2;
    command.crank_target_rate_rad_s = 6.;
    const auto torques =
        plant.controller->compute(plant.data, command, true, .00125, false);
    bt::oracle::Node node(
        node_id("test_spindle_coasting_command_matches_python"));
    compare_compute(node, torques, *plant.controller);
    node.done();
}

void test_spindle_compute_posture_matches_python() {
    rider::RiderPosture offset;
    offset.torso_lean_rad = -.2;
    offset.pelvis_offset_m = spindle::Vec2{-.05, .1};
    rider::RiderPosture lean;
    lean.torso_lean_rad = .3;
    lean.pelvis_pitch_rad = .2;
    rider::RiderPosture stand;
    stand.pelvis_pitch_rad = -.4;
    stand.use_saddle = false;
    const std::array cases{
        std::pair{rider::RiderPosture{}, "posture0"},
        std::pair{lean, "posture1"},
        std::pair{offset, "posture2"},
        std::pair{stand, "posture3"},
    };
    for (const auto &[posture, id] : cases) {
        Plant plant = make_plant();
        stage(plant, 1.9, 7.);
        rider::RiderCommand command;
        command.posture = posture;
        command.mean_crank_torque_nm = 70.;
        const auto torques = plant.controller->compute(
            plant.data, command, true, .00125, false);
        bt::oracle::Node node(node_id(std::format(
            "test_spindle_compute_posture_matches_python[{}]", id)));
        compare_compute(node, torques, *plant.controller);
        node.done();
    }
}

// ---- activation / damped fixture -------------------------------------------

void test_spindle_activation_rejects_repeated_timestamp() {
    Plant plant = make_plant(true);
    bt::require(plant.controller->config().activation_tau_s > 0.,
                "damped fixture carries activation_tau_s > 0");
    stage(plant, .4, 5.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 60.;
    (void)plant.controller->compute(plant.data, command, true, .00125,
                                    false);
    bool threw = false;
    try {
        (void)plant.controller->compute(plant.data, command, true, .00125,
                                        false);
    } catch (const std::exception &) {
        threw = true;
    }
    bt::require(threw, "repeated timestamp must reject");
}

// tests/reference/test_native_spindle_controller.py::
// test_spindle_activation_and_passive_damping_match_python is dropped, not
// rescoped: the oracle node was captured at joint_passive_damping_nms_rad
// = 2.5, so its pinned torques/state embed the passive term spec Q13
// removed. With the damping leg gone the surviving assertions (activation
// evolve + saw_damping) pin no oracle arithmetic — activation filtering is
// already covered by test_spindle_activation_rejects_repeated_timestamp
// and the compute-family state comparisons.

// ---- initialize / envelope / write ----------------------------------------

void test_spindle_initialize_matches_python() {
    Plant plant = make_plant();
    wbuf(plant.data->qpos, nq(plant.model))[joint_qposadr(
        plant.model, "crank_spin")] = 1.3;
    mj_forward(plant.model, plant.data);
    plant.controller->initialize(plant.data);
    bt::oracle::Node node(node_id("test_spindle_initialize_matches_python"));
    node.f64_close(buf(plant.data->qpos, nq(plant.model)), "qpos");
    node.done();
}

void test_spindle_initialize_velocity_matches_python() {
    Plant plant = make_plant();
    wbuf(plant.data->qpos, nq(plant.model))[joint_qposadr(
        plant.model, "crank_spin")] = .8;
    wbuf(plant.data->qvel, nv(plant.model))[joint_dofadr(
        plant.model, "crank_spin")] = 9.0;
    mj_forward(plant.model, plant.data);
    plant.controller->initialize_velocity(plant.data);
    bt::oracle::Node node(
        node_id("test_spindle_initialize_velocity_matches_python"));
    node.f64_close(buf(plant.data->qvel, nv(plant.model)),
                   "qvel");
    node.done();
}

void test_spindle_envelope_forces_match_python() {
    Plant plant = make_plant();
    plant.controller->initialize(plant.data);
    const auto [force, stored] =
        plant.controller->envelope_forces(plant.data);
    bt::oracle::Node node(
        node_id("test_spindle_envelope_forces_match_python"));
    node.f64_close(force, "force");
    const std::array<double, 1> scalar{stored};
    node.f64_close(scalar, "stored_j");
    node.done();
}

void test_spindle_write_matches_python() {
    Plant plant = make_plant();
    stage(plant, .5, 0.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 70.;
    const auto torques =
        plant.controller->compute(plant.data, command, true, .00125, false);
    plant.controller->write(plant.data, torques);
    bt::oracle::Node node(node_id("test_spindle_write_matches_python"));
    node.f64_close(buf(plant.data->ctrl, nu(plant.model)), "ctrl");
    node.done();
}

// ---- state round trip / solved effort --------------------------------------

void test_spindle_state_round_trip() {
    Plant plant = make_plant();
    stage(plant, .9, 4.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 55.;
    (void)plant.controller->compute(plant.data, command, true, .00125,
                                    false);
    bt::oracle::Node node(node_id("test_spindle_state_round_trip"));
    // The recorded document is both the decode input and the re-encode
    // oracle: restore() then encode_spindle_state() must reproduce it
    // byte-for-byte — modulo the pre-Q13 legacy members the tolerant
    // decode accepts-and-ignores and the re-encode drops (R3). The
    // surviving content is still a lossless round-trip.
    const auto document = node.bytes_input("restored_state");
    const std::string_view source(
        reinterpret_cast<const char *>(document.data()), document.size()); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
    plant.controller->restore(rider::decode_spindle_state(
        bio::parse_json(source), "restored_state"));
    const std::string reencoded = state_document(*plant.controller);
    bio::Value expected = bio::parse_json(source);
    strip_removed_passive_keys(expected);
    const std::string expected_text = bio::canonical_json(expected);
    bt::require(
        std::ranges::equal(
            byte_span(reencoded), byte_span(expected_text)),
        "restore+encode reproduces the surviving wire byte-for-byte");
    mj_resetData(plant.model, plant.data);
    plant.data->time = .00125;
    wbuf(plant.data->qpos, nq(plant.model))[joint_qposadr(
        plant.model, "crank_spin")] = 1.05;
    wbuf(plant.data->qvel, nv(plant.model))[joint_dofadr(
        plant.model, "crank_spin")] = 5.;
    mj_forward(plant.model, plant.data);
    const auto torques =
        plant.controller->compute(plant.data, command, true, .00125, false);
    compare_compute(node, torques, *plant.controller);
    node.done();
}

void test_spindle_solved_effort_matches_python() {
    Plant plant = make_plant();
    stage(plant, .6, 8.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 75.;
    const auto torques =
        plant.controller->compute(plant.data, command, true, .00125, false);
    plant.controller->write(plant.data, torques);
    const std::span<const double> qpos_view =
        buf(plant.data->qpos, nq(plant.model));
    const std::vector<double> incoming_qpos(qpos_view.begin(),
                                            qpos_view.end());
    const std::span<const double> qvel_view =
        buf(plant.data->qvel, nv(plant.model));
    const std::vector<double> incoming_qvel(qvel_view.begin(),
                                            qvel_view.end());
    mj_forward(plant.model, plant.data);
    const auto diagnostics = plant.controller->solved_effort(
        plant.data, incoming_qpos, incoming_qvel, .00125);
    bt::oracle::Node node(
        node_id("test_spindle_solved_effort_matches_python"));
    const auto delivered =
        torque_vector(diagnostics.active_delivered_nm, *plant.controller);
    node.f64_close(delivered, "solved.delivered");
    // The passive_power_w / passive_work_step_j fields are removed (spec
    // Q13); the recorded solved.powers slots for them were captured at
    // damping = 0 and pin 0.0, so literal zeroes preserve the 4-wide label
    // byte-for-byte while asserting no passive power can ever emerge.
    const double powers[] = { // NOLINT(cppcoreguidelines-avoid-c-arrays,modernize-avoid-c-arrays) bounded label buffer for snprintf
        diagnostics.positive_power_w,
        0.,
        diagnostics.positive_work_step_j.value_or(0.),
        0.,
    };
    node.f64_close(powers, "solved.powers");
    bio::Value::Array names;
    if (diagnostics.strength_violations)
        for (const auto &name : *diagnostics.strength_violations)
            names.push_back(bio::Value{std::string(name)});
    bio::Value::Object object;
    object.emplace_back("violations", bio::Value{std::move(names)});
    const std::string doc =
        bio::canonical_json(bio::Value{std::move(object)});
    const std::array<std::size_t, 1> shape{doc.size()};
    node.bytes(byte_span(doc), "|u1", shape, "solved.violations");
    expect_state_close(node, state_document(*plant.controller),
                       "solved.state");
    node.done();
}

void test_spindle_reset_activation_matches_python() {
    Plant plant = make_plant();
    stage(plant, .2, 3.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 50.;
    (void)plant.controller->compute(plant.data, command, true, .00125,
                                    false);
    plant.controller->reset_activation();
    const auto state = plant.controller->state();
    bt::require(std::ranges::all_of(state.active_state,
                                    [](double v) { return v == 0.; }),
                "reset zeroes the activation filter state");
    bt::require(!state.activation_time_s.has_value(),
                "reset clears activation_time_s");
    bt::oracle::Node node(
        node_id("test_spindle_reset_activation_matches_python"));
    expect_state_close(node, state_document(*plant.controller),
                       "reset_state");
    node.done();
}

// ---- strength probes / rejection contracts ---------------------------------

void test_spindle_strength_probes_match_python() {
    Plant plant = make_plant();
    stage(plant, .4, 0.);
    std::vector<double> anatomical;
    std::vector<double> capacities;
    const std::array<std::array<double, 2>, 3> probes{
        {{0., 30.}, {2.5, -20.}, {-1.5, 45.}}};
    for (const auto &joint : plant.controller->joints()) {
        const double angle = plant.controller->anatomical_joint_angle(
            joint.name,
            buf(plant.data->qpos, nq(plant.model))
                [static_cast<std::size_t>(joint.qpos_adr)]);
        anatomical.push_back(angle);
        for (const auto &[velocity, torque] : probes)
            capacities.push_back(plant.controller->strength_capacity(
                joint.name, angle, velocity, torque));
    }
    std::vector<double> requested;
    for (const auto &joint : plant.controller->joints())
        requested.push_back(joint.name.find("knee") != std::string::npos
                                ? 120. : -90.);
    const std::span<const double> qpos_view =
        buf(plant.data->qpos, nq(plant.model));
    const std::vector<double> qpos(qpos_view.begin(), qpos_view.end());
    const std::span<const double> qvel_view =
        buf(plant.data->qvel, nv(plant.model));
    const std::vector<double> qvel(qvel_view.begin(), qvel_view.end());
    const auto [clipped, limited] = plant.controller->strength_limited(
        requested, qpos, qvel);
    bt::oracle::Node node(
        node_id("test_spindle_strength_probes_match_python"));
    node.f64_close(anatomical, "anatomical");
    node.f64_close(capacities, "capacity");
    node.f64_close(clipped, "clipped");
    bio::Value::Array names;
    for (const auto &name : limited)
        names.push_back(bio::Value{std::string(name)});
    bio::Value::Object object;
    object.emplace_back("limited", bio::Value{std::move(names)});
    const std::string doc =
        bio::canonical_json(bio::Value{std::move(object)});
    const std::array<std::size_t, 1> shape{doc.size()};
    node.bytes(byte_span(doc), "|u1", shape, "limited");
    node.done();
}

void test_spindle_strength_rejects_malformed_state() {
    Plant plant = make_plant();
    stage(plant, .4, 0.);
    const std::span<const double> qpos_view =
        buf(plant.data->qpos, nq(plant.model));
    const std::vector<double> qpos(qpos_view.begin(), qpos_view.end());
    const std::span<const double> qvel_view =
        buf(plant.data->qvel, nv(plant.model));
    const std::vector<double> qvel(qvel_view.begin(), qvel_view.end());
    const std::vector<double> flat(plant.controller->joints().size(), 0.);
    bt::require(throws_std([&] {
                    (void)plant.controller->strength_limited(
                        flat, std::span<const double>(qpos).subspan(0, 2),
                        qvel);
                }),
                "narrow qpos rejects");
    bt::require(throws_std([&] {
                    (void)plant.controller->strength_limited(
                        flat, qpos,
                        std::span<const double>(qvel).subspan(0, 2));
                }),
                "narrow qvel rejects");
    bt::require(throws_std([&] {
                    (void)plant.controller->solved_effort(
                        plant.data,
                        std::span<const double>(qpos).subspan(0, 2), qvel,
                        .00125);
                }),
                "solved_effort narrow qpos rejects");
    bt::require(throws_std([&] {
                    (void)plant.controller->solved_effort(
                        plant.data, qpos,
                        std::span<const double>(qvel).subspan(0, 2), .00125);
                }),
                "solved_effort narrow qvel rejects");
}

void test_spindle_lookups_reject_unknown_joints() {
    Plant plant = make_plant();
    stage(plant, .4, 0.);
    for (const char *name : {"rider_ankle_front", "rider_wrist"})
        bt::require(throws_std([&] {
                        plant.controller->write(
                            plant.data, rider::NamedEntries<double>{
                                            {name, 0.}});
                    }),
                    std::string("write rejects unknown joint ") + name);
    bt::require(throws_std([&] {
                    (void)plant.controller->anatomical_joint_angle(
                        "rider_wrist", 0.);
                }),
                "anatomical_joint_angle rejects unknown joint");
    bt::require(throws_std([&] {
                    (void)plant.controller->strength_capacity(
                        "rider_wrist", 0., 0., 5.);
                }),
                "strength_capacity rejects unknown joint");
}

void test_spindle_probe_rejects_bad_input() {
  Plant const plant = make_plant();
  bt::require(throws_std([&] {
                (void)rider::decode_rider_command(
                    bio::Value{bio::Value::Object{}}, "command");
              }),
              "empty command document rejects");
  bt::require(throws_std(
                  [&] {
                    bio::Value::Object command;
                    command.emplace_back(
                        "mean_crank_torque_nm",
                        bio::Value{bio::Number{"-1."}});
                    (void)rider::decode_rider_command(
                        bio::Value{std::move(command)}, "command");
                  }),
              "negative mean crank torque rejects");
  bt::require(throws_std([&] {
                bio::Value::Object state;
                state.emplace_back("enabled", bio::Value{true});
                (void)rider::decode_spindle_state(bio::Value{std::move(state)},
                                                  "state");
              }),
              "partial state document rejects");
  // A schema-valid non-spindle attachment routes to the ported legacy
  // controller rather than rejecting at decode — the wire decode marks
  // config.legacy for the runtime's controller dispatch (the python
  // node's adapter-level construction is exercised through the runtime
  // ports, not this wire contract).
  bio::Value const doc =
      bio::parse_json(bt::text(bt::fixture("spindle/pinned/config.json")));
  bio::Value rider_doc = member(doc, "rider_controller");
  set_string(rider_doc, "pedal_attachment", "flat");
  const auto flat_config = rider::decode_spindle_config(rider_doc);
  bt::require(flat_config.legacy.has_value() &&
                  flat_config.legacy->pedal_attachment == "flat",
              "flat attachment decodes into the legacy route");
  // A mode outside the schema set is still rejected at decode.
  set_string(rider_doc, "pedal_attachment", "pin");
  bt::require(
      throws_std([&] { (void)rider::decode_spindle_config(rider_doc); }),
      "pedal_attachment=pin rejects");
  // The removed joint_passive_damping_nms_rad knob is accepted only when
  // the recorded value is inert: a recorded 0.0 decodes, but an explicit
  // null is never inert in a rider_controller payload — the unset knob
  // resolved through the joint_kd_nms_rad fallback on articulated
  // hinges, so decode rejects it like a nonzero value.
  bio::Value damping_doc = member(doc, "rider_controller");
  auto &damping_members = std::get<bio::Value::Object>(damping_doc.data);
  damping_members.emplace_back("joint_passive_damping_nms_rad",
                               bio::Value{bio::Number{"0.0"}});
  (void)rider::decode_spindle_config(damping_doc);
  damping_members.back().second = bio::Value{};
  try {
    (void)rider::decode_spindle_config(damping_doc);
    bt::require(false,
                "recorded null joint_passive_damping_nms_rad rejects");
  } catch (const std::invalid_argument &failure) {
    bt::require(std::string_view(failure.what())
                        .find("joint_passive_damping_nms_rad") !=
                    std::string_view::npos,
                "the null damping rejection names the removed key");
  }
  bt::require(throws_std([&] {
                (void)rider::decode_spindle_config(
                    bio::Value{bio::Value::Object{}});
              }),
              "missing config members reject");
}
} // namespace

// bt::run wraps the test body in try/catch(...) — failures are reported, nothing escapes main.
// The dispatcher's inlined callees keep every plant's exception-cleanup slot
// live, so the frame legitimately exceeds the 8KB guard whose target is the
// per-step realtime path — same rationale as test_contracts' large cases.
// cppcheck-suppress throwInEntryPoint
NATIVE_DIAG_PUSH
NATIVE_DIAG_IGNORE("-Wframe-larger-than")
int main() {
    return bt::run([] {
        test_spindle_waveform_matches_python();
        test_effort_ceiling_matches_python();
        test_spindle_waveform_rejects_bad_inputs();
        test_effort_ceiling_rejects_bad_inputs();
        test_spindle_compute_matches_python();
        test_spindle_rear_chain_effort_matches_python();
        test_spindle_disabled_command_matches_python();
        test_spindle_steady_state_bypasses_activation();
        test_spindle_coasting_command_matches_python();
        test_spindle_compute_posture_matches_python();
        test_spindle_activation_rejects_repeated_timestamp();
        // test_spindle_activation_and_passive_damping_match_python —
        // dropped above (spec Q13: the damped oracle is not reproducible).
        test_spindle_initialize_matches_python();
        test_spindle_initialize_velocity_matches_python();
        test_spindle_envelope_forces_match_python();
        test_spindle_write_matches_python();
        test_spindle_state_round_trip();
        test_spindle_solved_effort_matches_python();
        test_spindle_reset_activation_matches_python();
        test_spindle_strength_probes_match_python();
        test_spindle_strength_rejects_malformed_state();
        test_spindle_lookups_reject_unknown_joints();
        test_spindle_probe_rejects_bad_input();
    });
}
NATIVE_DIAG_POP
