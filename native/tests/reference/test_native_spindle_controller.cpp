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
#include "rider/equality_reactions.hpp"
#include "rider/spindle_controller.hpp"
#include "rider/spindle_math.hpp"
#include "rider/state_codec.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <format>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <numeric>
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

Plant make_plant(std::string_view pedal_attachment = {},
                 const std::function<void(rider::SpindleConfig &)> &tune = {}) {
    Plant plant;
    const auto mjb = bt::fixture("spindle/pinned/model.mjb");
    plant.model = mj_loadModel(mjb.c_str(), nullptr);
    bt::require(plant.model != nullptr, "spindle/pinned/model.mjb loads");
    plant.data = mj_makeData(plant.model);
    bt::require(plant.data != nullptr, "mj_makeData");
    bio::Value const doc =
        bio::parse_json(bt::text(bt::fixture("spindle/pinned/config.json")));
    bio::Value rider_doc = member(doc, "rider_controller");
    if (!pedal_attachment.empty())
        set_string(rider_doc, "pedal_attachment", pedal_attachment);
    rider::SpindleConfig config = rider::decode_spindle_config(rider_doc);
    // The reposition law's profile inputs do not cross the rider_controller
    // wire; the runtime assembly injects them from drive.pedaling and
    // seated_climb. The pinned plant declares the calibrated 12/60 baseline.
    config.reposition_damping_nms_rad = 12.;
    config.reposition_torque_limit_nm = 60.;
    if (tune)
        tune(config);
    plant.controller = std::make_unique<rider::SpindleController>(
        plant.model,
        rider::decode_spindle_pose(member(doc, "pose"), "config.pose"),
        std::move(config),
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

void expect_state_close(bt::oracle::Node &node, const std::string &document,
                        std::string_view label) {
  const auto expected_bytes = node.bytes_input(label);
  const std::string_view source(
      reinterpret_cast<const char *>(expected_bytes.data()), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
      expected_bytes.size());
  std::string path;
  const bool close =
      json_close(bio::parse_json(document), bio::parse_json(source), path);
  // Record mode keeps the recorded document while it still matches within
  // tolerance, so an unchanged state does not churn the golden.
  if (bt::oracle::Node::recording()) {
    if (!close) node.record_consumed(byte_span(document));
    return;
  }
  bt::require(close, std::string(label) +
                         ": state diverges beyond platform tolerance at" +
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

// physics-2026-11-05 plan task 3 («Действует» 9) read with the ruling's
// 2026-11-07/08 refinement: with nothing asked of the muscles
// (mean_crank_torque_nm = 0 — the coasting branch, no weld sensor so the
// hold servo cannot learn to lift the leg) and a free crank — the pinned
// plant's crank carries nothing downstream, which is exactly an open
// one-way clutch — each leg rests its own weight on its pedal. The
// muscles lift only what exceeds the rising leg's weight («Действует» 9:
// its press stays inside [0, leg weight]). Under the 2026-11-10 ruling a
// coasting rider's legs rest on the pedals with exactly that weight —
// but the two legs do not weigh the same, so on this drivetrain-less
// pinned crank the residual weight torque rotates the shaft until the
// two projections balance (an unbalanced wheel settling heavy-side-down)
// and then stops. The physical claim the test keeps: resting legs never
// *drive* the crank — the motion is bounded equilibrium-seek that
// decays to rest, not an accumulating spin-up. (The pre-8e8b0b0d
// feedforward version spun ~1.8 rad forward and kept going; the
// 2026-11-06 hold-only version stayed inside .5 rad only because the
// presses it requested were symmetric by accident.)
void test_free_crank_stays_held_under_balanced_leg_weights() {
    Plant plant = make_plant();
    // The staged pose has the front pedal at 3 o'clock — crank_split's
    // leg_shares hands the downstroke to the front leg (cos(phase) > 0).
    stage(plant, 0., 0.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 0.;
    command.crank_target_rate_rad_s = 0.;
    const auto dof = joint_dofadr(plant.model, "crank_spin");
    const auto qpos = joint_qposadr(plant.model, "crank_spin");
    const double dt = plant.model->opt.timestep;
    // The pinned crank is the open-clutch ideal — nothing downstream
    // drives or holds it — but an inertia-free shaft escapes under a few
    // N.m of press and the weld springs tear. The bench loads it with a
    // weak viscous term (a dynamometer drag) so any residual imbalance
    // reads as a bounded rate instead of a runaway.
    constexpr double kLoadNms = 1.;
    std::vector<double> rates;
    double max_excursion = 0.;
    for (int i = 0; i < 3200; ++i) { // 4 s of free cranking
        const auto torques = plant.controller->compute(
            plant.data, command, true, dt, false);
        plant.controller->write(plant.data, torques);
        wbuf(plant.data->qfrc_applied, nv(plant.model))[dof] =
            -kLoadNms * buf(plant.data->qvel, nv(plant.model))[dof];
        mj_step(plant.model, plant.data);
        rates.push_back(buf(plant.data->qvel, nv(plant.model))[dof]);
        max_excursion =
            std::max(max_excursion,
                     std::abs(buf(plant.data->qpos, nq(plant.model))[qpos]));
    }
    const double peak = *std::ranges::max_element(rates);
    double settled = 0.;
    for (const double rate : std::span(rates).last(400))
        settled = std::max(settled, std::abs(rate));
    std::cerr << std::format(
        "free-crank peak rate {:.4f} rad/s, settled |rate| {:.4f} rad/s, "
        "|excursion| {:.3f} rad\n",
        peak, settled, max_excursion);
    // A driven crank accumulates unboundedly; a resting one seeks the
    // pose where the two unequal weight projections cancel and stops.
    // The excursion stays far inside half a turn and the last half
    // second of the run is at rest — the residual peak is weld-spring
    // ringing during the swing, not drive.
    bt::require(max_excursion <= std::numbers::pi,
                "resting leg weights seek a bounded crank equilibrium — "
                "the free crank never completes half a turn");
    bt::require(settled <= .05,
                "resting leg weights leave the free crank at rest — "
                "no persistent drive");
}

// User ruling 2026-11-10 (supersedes the 2026-11-06 hold-only ruling):
// coasting is a rider who stops pushing — the coast branch of
// populate_leg_torques applies no crank torque at all: no rate damper,
// no press-cancel hold, no learned coast_hold_nm_. The diagnostics'
// crank_task_nm must read exactly 0, and the shared hold must be zero
// on every coasting tick so nothing carries into the resume.
void test_spindle_coasting_applies_no_crank_torque() {
    // Trailing crank (below the decaying stop trajectory), overrunning
    // crank (above it), stopped crank and the creeping band the old
    // damper gated on: every coasting answer is no crank task.
    for (const double rate : {4., 8., 0., .05, .2}) {
        Plant plant = make_plant();
        stage(plant, .35, rate);
        rider::RiderCommand command;
        command.mean_crank_torque_nm = 0.;
        command.crank_target_phase_rad = 1.2;
        command.crank_target_rate_rad_s = 6.;
        (void)plant.controller->compute(plant.data, command, true,
                                        .00125, false);
        const double task = plant.controller->state()
                                .allocation_diagnostics.crank_task_nm;
        bt::require(task == 0.,
                    std::format("coasting crank at {:.2f} rad/s: servo "
                                "task {:.6f} N.m — a coasting rider "
                                "applies no crank torque", rate, task));
        bt::require(plant.controller->state().coast_hold_nm == 0.,
                    std::format("coasting crank at {:.2f} rad/s: "
                                "coast_hold_nm_ stays zero", rate));
    }
    // A weld-measured pull-back press does not wake the branch either.
    {
        Plant plant = make_plant();
        stage(plant, .35, 4.);
        rider::RiderCommand command;
        command.mean_crank_torque_nm = 0.;
        command.crank_target_phase_rad = 1.2;
        command.crank_target_rate_rad_s = 6.;
        (void)plant.controller->compute(plant.data, command, true,
                                        .00125, false, std::nullopt,
                                        false, -5.);
        const double task = plant.controller->state()
                                .allocation_diagnostics.crank_task_nm;
        bt::require(task == 0.,
                    std::format("pull-back weld press: servo task "
                                "{:.6f} N.m — no crank torque", task));
    }
    // A hold carried over from pedaling clears on the first coast tick
    // and stays cleared — nothing is handed to the resume.
    {
        Plant plant = make_plant();
        stage(plant, .35, 4.);
        auto snapshot = plant.controller->state();
        snapshot.coast_hold_nm = 7.;
        plant.controller->commit_state(std::move(snapshot));
        rider::RiderCommand command;
        command.mean_crank_torque_nm = 0.;
        command.crank_target_phase_rad = 1.2;
        command.crank_target_rate_rad_s = 6.;
        for (int i = 0; i < 3; ++i)
            (void)plant.controller->compute(plant.data, command, true,
                                            .00125, false);
        const auto state = plant.controller->state();
        bt::require(state.coast_hold_nm == 0. &&
                        state.allocation_diagnostics.crank_task_nm == 0.,
                    "a carried-in hold clears on the first coast tick");
    }
}

// The crank-reposition manoeuvre's dedicated branch (2026-11-16 plan
// task 6): a zero-effort command carrying crank_reposition backpedals
// the crank toward its negative rate goal —
// raw = config.reposition_damping_nms_rad * (crank_target_rate_rad_s - rate),
// clamped inside ±config.reposition_torque_limit_nm. The coasting press servo
// would drive that same torque to zero, so the branch never consults
// it: the weld reading changes nothing and the carried pedaling hold
// survives untouched for the resume.
void test_spindle_reposition_backpedals_toward_goal() {
    Plant plant = make_plant();
    stage(plant, .35, 0.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 0.;
    command.crank_target_phase_rad = 0.;
    command.crank_target_rate_rad_s = -3.;
    command.crank_reposition = true;
    (void)plant.controller->compute(plant.data, command, true,
                                    .00125, false);
    const double task = plant.controller->state()
                            .allocation_diagnostics.crank_task_nm;
    bt::require(task < 0. &&
                    task >= -plant.controller->config().reposition_torque_limit_nm,
                std::format("reposition task {:.6f} N.m: backward and "
                            "inside the torque limit", task));
    bt::require(task == -36.,
                std::format("damping law 12 N.m.s/rad over -3 rad/s at a "
                            "stopped crank lands exactly, got {:.6f}",
                            task));
    // A weld reading equal to the delivered torque does not cancel the
    // request — the press servo never runs in this branch.
    (void)plant.controller->compute(plant.data, command, true,
                                    .00125, false, std::nullopt, false,
                                    task);
    const double held = plant.controller->state()
                            .allocation_diagnostics.crank_task_nm;
    bt::require(held == task,
                "the weld reading leaves the reposition task unchanged");
    // A forward-rolling crank saturates the backpedal request at the
    // declared limit, never beyond.
    {
        Plant fast = make_plant();
        stage(fast, .35, 40.);
        (void)fast.controller->compute(fast.data, command, true,
                                       .00125, false);
        bt::require(fast.controller->state()
                            .allocation_diagnostics.crank_task_nm ==
                        -fast.controller->config().reposition_torque_limit_nm,
                    "the reposition torque clamps at the limit");
    }
    // The pedaling hold is carried through untouched — the branch
    // neither zeroes it like the coast branch nor adds it to the task.
    {
        Plant carried = make_plant();
        stage(carried, .35, 0.);
        auto snapshot = carried.controller->state();
        snapshot.coast_hold_nm = 7.;
        carried.controller->commit_state(std::move(snapshot));
        (void)carried.controller->compute(carried.data, command, true,
                                          .00125, false);
        const auto state = carried.controller->state();
        bt::require(state.coast_hold_nm == 7. &&
                        state.allocation_diagnostics.crank_task_nm == -36.,
                    "the carried pedaling hold survives untouched");
    }
}

// The command document (2026-11-16 plan task 6): crank_reposition is
// an optional member the decoder admits only beside a zero-effort
// command, and the encoder emits it only while the manoeuvre runs —
// a disengaged command keeps the historical five-key document.
void test_spindle_reposition_command_wire() {
    const auto document = [](std::string_view extra = {},
                             std::string_view effort = "0.0",
                             std::string_view phase = "0.0",
                             std::string_view rate = "-3.0") {
        return bio::parse_json(
            std::string(R"({"mean_crank_torque_nm":)") +
            std::string(effort) +
            R"(,"enabled":true,"posture":{"torso_lean_rad":0.1,"pelvis_pitch_rad":0.0,"pelvis_offset_m":null,"use_saddle":true},"crank_target_phase_rad":)" +
            std::string(phase) +
            R"(,"crank_target_rate_rad_s":)" + std::string(rate) +
            std::string(extra) + "}");
    };
    const rider::RiderCommand engaged = rider::decode_rider_command(
        document(R"(,"crank_reposition":true)"), "command");
    bt::require(engaged.crank_reposition,
                "the optional member decodes");
    const rider::RiderCommand disengaged =
        rider::decode_rider_command(document(), "command");
    bt::require(!disengaged.crank_reposition,
                "the absent member decodes to false");
    // A present phase goal plus effort trips the coasting-goal guard
    // before the reposition one — both rejections exist independently.
    bt::require(throws_std([&] {
                    (void)rider::decode_rider_command(
                        document(R"(,"crank_reposition":true)", "5.0"),
                        "command");
                }),
                "a coasting goal with pedaling effort rejects");
    // With the phase goal absent the reposition+effort guard is the one
    // that must fire — it is unreachable through the document above.
    bt::require(throws_std([&] {
                    (void)rider::decode_rider_command(
                        document(R"(,"crank_reposition":true)", "5.0",
                                 "null"),
                        "command");
                }),
                "reposition with pedaling effort rejects");
    // The backpedal owns the rate lane while the flag is up: a
    // nonnegative caller rate is malformed beside an engaged manoeuvre.
    for (const char *rate : {"0.0", "1.5"}) {
        bt::require(throws_std([&] {
                        (void)rider::decode_rider_command(
                            document(R"(,"crank_reposition":true)", "0.0",
                                     "null", rate),
                            "command");
                    }),
                    "reposition with a nonnegative crank rate rejects");
    }
    // The negative-rate goal beside a null phase is the valid engaged
    // document — the guard rejects the malformed lanes only.
    const rider::RiderCommand goalless = rider::decode_rider_command(
        document(R"(,"crank_reposition":true)", "0.0", "null"), "command");
    bt::require(goalless.crank_reposition &&
                    !goalless.crank_target_phase_rad.has_value() &&
                    goalless.crank_target_rate_rad_s < 0.,
                "an engaged goalless reposition command decodes");
    bt::require(throws_std([&] {
                    (void)rider::decode_rider_command(
                        document(R"(,"crank_reposition":1)"), "command");
                }),
                "a non-bool crank_reposition rejects");
    // The encoder emits the member only when engaged.
    const bio::Value encoded = rider::encode_rider_command(engaged);
    const bio::Value *const flag = bio::find_member(
        std::get<bio::Value::Object>(encoded.data), "crank_reposition");
    const auto *const flag_bool =
        flag != nullptr ? std::get_if<bool>(&flag->data) : nullptr;
    bt::require(flag_bool != nullptr && *flag_bool,
                "the engaged command carries crank_reposition: true");
    const bio::Value quiet = rider::encode_rider_command(disengaged);
    bt::require(bio::find_member(std::get<bio::Value::Object>(quiet.data),
                                 "crank_reposition") == nullptr,
                "the disengaged command keeps the five-key shape");
    // The flag survives the full encode-decode pass.
    bt::require(rider::decode_rider_command(encoded, "command")
                    .crank_reposition,
                "the engaged flag round-trips");
}

// «Действует» 9's leg weight at the pedal — the same γ·leg-gravity
// quantity the seated ride's LegWeightProbe measures
// (test_seated_pedaling_cycle.cpp): the pedal-normal share of each
// leg's own gravity at the current pose. User ruling 2026-11-10: a
// coasting rider's legs rest on the pedals with exactly this weight —
// relaxed, not held weightless. Both controller paths must request it.
struct LegWeightProbe {
    std::array<spindle::LegLoopGeometry, 2> geom{};
    std::array<int, 2> hip_joints{-1, -1};
    std::array<int, 2> knee_joints{-1, -1};
    std::array<int, 2> pedal_bodies{-1, -1};
    std::array<std::array<int, 3>, 2> leg_bodies{};
    int frame_body = -1;
    int crank_joint = -1;
};

LegWeightProbe leg_weight_probe(const mjModel *m, const bio::Value &doc) {
    LegWeightProbe probe;
    const auto pose =
        rider::decode_spindle_pose(member(doc, "pose"), "config.pose");
    const auto xz = [](const std::array<double, 3> &v) {
        return spindle::Vec2{v[0], v[2]};
    };
    probe.frame_body = mj_name2id(m, mjOBJ_BODY, "frame");
    probe.crank_joint = mj_name2id(m, mjOBJ_JOINT, "crank_spin");
    for (std::size_t s = 0; s < 2; ++s) {
        const char *side = s == 0 ? "front" : "rear";
        const auto named = [&](const char *part) {
            return std::string(part) + side;
        };
        probe.geom[s] = spindle::leg_loop_geometry(
            xz(pose.hip), xz(s == 0 ? pose.knee_front : pose.knee_rear),
            xz(s == 0 ? pose.pedal_front : pose.pedal_rear), s == 0);
        probe.hip_joints[s] =
            mj_name2id(m, mjOBJ_JOINT, named("rider_hip_").c_str());
        probe.knee_joints[s] =
            mj_name2id(m, mjOBJ_JOINT, named("rider_knee_").c_str());
        probe.pedal_bodies[s] =
            mj_name2id(m, mjOBJ_BODY, named("pedal_").c_str());
        probe.leg_bodies[s] = {
            mj_name2id(m, mjOBJ_BODY, named("rider_thigh_").c_str()),
            mj_name2id(m, mjOBJ_BODY, named("rider_shank_").c_str()),
            mj_name2id(m, mjOBJ_BODY, named("rider_foot_").c_str())};
    }
    return probe;
}

std::array<double, 2> leg_weights_at_pedals(const LegWeightProbe &probe,
                                            const mjModel *m,
                                            const mjData *d) {
    const auto xmat = buf(d->xmat, static_cast<std::size_t>(9 * m->nbody));
    const auto xanchor =
        buf(d->xanchor, static_cast<std::size_t>(3 * m->njnt));
    const auto xpos = buf(d->xpos, static_cast<std::size_t>(3 * m->nbody));
    const auto fb = 9 * static_cast<std::size_t>(probe.frame_body);
    const auto cj = 3 * static_cast<std::size_t>(probe.crank_joint);
    const auto to_frame_xz = [&](std::span<const double> p,
                                 std::size_t base) -> spindle::Vec2 {
        const double dx = p[base] - xanchor[cj];
        const double dy = p[base + 1] - xanchor[cj + 1];
        const double dz = p[base + 2] - xanchor[cj + 2];
        return {xmat[fb] * dx + xmat[fb + 3] * dy + xmat[fb + 6] * dz,
                xmat[fb + 2] * dx + xmat[fb + 5] * dy + xmat[fb + 8] * dz};
    };
    const auto width = static_cast<std::size_t>(m->nv);
    const auto gravity = buf(std::data(m->opt.gravity), 3);
    const auto body_mass =
        buf(m->body_mass, static_cast<std::size_t>(m->nbody));
    const auto dofadr =
        ibuf(m->jnt_dofadr, static_cast<std::size_t>(m->njnt));
    std::vector<mjtNum> jac(3 * width);
    std::array<double, 2> out{};
    for (std::size_t s = 0; s < 2; ++s) {
        const auto pedal = to_frame_xz(
            xpos, 3 * static_cast<std::size_t>(probe.pedal_bodies[s]));
        const auto hip = to_frame_xz(
            xanchor, 3 * static_cast<std::size_t>(probe.hip_joints[s]));
        const auto gamma =
            spindle::leg_vertical_gamma(probe.geom[s], hip, pedal);
        const std::array<std::size_t, 2> dofs{
            static_cast<std::size_t>(dofadr[static_cast<std::size_t>(
                probe.hip_joints[s])]),
            static_cast<std::size_t>(dofadr[static_cast<std::size_t>(
                probe.knee_joints[s])])};
        std::array<double, 2> leg_gravity{0., 0.};
        for (const int b : probe.leg_bodies[s]) {
            mj_jacBodyCom(m, d, jac.data(), nullptr, b);
            const double mass = body_mass[static_cast<std::size_t>(b)];
            for (std::size_t i = 0; i < 2; ++i) {
                const auto dof = dofs[i];
                leg_gravity[i] -=
                    mass * (gravity[0] * jac[dof] +
                            gravity[1] * jac[width + dof] +
                            gravity[2] * jac[2 * width + dof]);
            }
        }
        out[s] = gamma[0] * leg_gravity[0] + gamma[1] * leg_gravity[1];
    }
    return out;
}

// The spindle branch: a coasting rider's legs rest on the pedals with
// their own weight. The connect_foot_* equalities are the sole↔pedal
// attachments — after the press servos settle, the force each applies
// to the foot must equal the leg's own weight at the pedal.
void test_spindle_coasting_leg_rests_with_own_weight() {
    Plant plant = make_plant();
    bio::Value const doc =
        bio::parse_json(bt::text(bt::fixture("spindle/pinned/config.json")));
    const auto probe = leg_weight_probe(plant.model, doc);
    stage(plant, 0., 0.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 0.;
    command.crank_target_phase_rad = 0.;
    command.crank_target_rate_rad_s = 0.;
    const double dt = plant.model->opt.timestep;
    for (int i = 0; i < 400; ++i) {
        const auto torques = plant.controller->compute(
            plant.data, command, true, dt, false);
        plant.controller->write(plant.data, torques);
        mj_step(plant.model, plant.data);
    }
    mj_forward(plant.model, plant.data);
    const auto expected =
        leg_weights_at_pedals(probe, plant.model, plant.data);
    const auto crank_rate = buf(
        plant.data->qvel, nv(plant.model))[joint_dofadr(plant.model,
                                                        "crank_spin")];
    const rider::EqualityReactions reactions(plant.model);
    for (const auto s : {0U, 1U}) {
        const int eq = mj_name2id(
            plant.model, mjOBJ_EQUALITY,
            s == 0U ? "connect_foot_front" : "connect_foot_rear");
        bt::require(eq >= 0, "connect_foot equality resolves");
        const auto force = reactions.force_on_rider(plant.data, eq);
        bt::require(force.size() == 3,
                    "connect equality carries three rows");
        const double press = force[2];
        std::cerr << std::format(
            "coasting rest press[{}]={:.3f} N vs leg weight {:.3f} N "
            "(crank rate {:.4f} rad/s)\n",
            s, press, expected[s], crank_rate);
        bt::require(std::abs(press - expected[s]) <= .05 * expected[s],
                    std::format(
                        "side {}: coasting leg rests with its own "
                        "weight — pedal holds {:.3f} N, leg weighs "
                        "{:.3f} N",
                        s, press, expected[s]));
    }
    // What the two unequal presses do to this drivetrain-less crank —
    // bounded equilibrium-seek, not drive — is the free-crank test's
    // claim; this test only pins the requested weights.
}

// The legacy branch (pedal_attachment "weld" on the same plant): the
// coast foot-force request of each leg is that leg's own weight, read
// off support_diagnostics.legacy->pedal_forces.
void test_legacy_coasting_leg_rests_with_own_weight() {
    Plant plant = make_plant("weld");
    bio::Value const doc =
        bio::parse_json(bt::text(bt::fixture("spindle/pinned/config.json")));
    const auto probe = leg_weight_probe(plant.model, doc);
    stage(plant, 0., 0.);
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 0.;
    command.crank_target_phase_rad = 0.;
    command.crank_target_rate_rad_s = 0.;
    const double dt = plant.model->opt.timestep;
    for (int i = 0; i < 4; ++i)
        (void)plant.controller->compute(plant.data, command, true, dt,
                                        false);
    const auto &diagnostics =
        plant.controller->state().support_diagnostics;
    bt::require(diagnostics.present && diagnostics.legacy.has_value(),
                "legacy support diagnostics present");
    const auto expected =
        leg_weights_at_pedals(probe, plant.model, plant.data);
    for (const auto s : {0U, 1U}) {
        const auto &force =
            diagnostics.legacy.value().pedal_forces[s]; // NOLINT(bugprone-unchecked-optional-access) engagement is asserted by the require above; .value() reports violations loudly
        const double press = -force[2];
        std::cerr << std::format(
            "legacy coasting rest press[{}]={:.3f} N vs leg weight "
            "{:.3f} N (request {{{:.3f}, {:.3f}, {:.3f}}})\n",
            s, press, expected[s], force[0], force[1], force[2]);
        bt::require(
            std::abs(press - expected[s]) <= .05 * expected[s] &&
                std::abs(force[0]) <= .05 * expected[s] &&
                std::abs(force[1]) <= .05 * expected[s],
            std::format(
                "side {}: coasting leg requests its own weight on the "
                "pedal — {{0, 0, -{:.3f}}}, got {{{:.3f}, {:.3f}, "
                "{:.3f}}}",
                s, expected[s], force[0], force[1], force[2]));
    }
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
    // byte-for-byte (canonical spelling).
    const auto document = node.bytes_input("restored_state");
    const std::string_view source(
        reinterpret_cast<const char *>(document.data()), document.size()); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast) byte buffer viewed as chars for the I/O API; the sanctioned aliasing bridge
    plant.controller->restore(rider::decode_spindle_state(
        bio::parse_json(source), "restored_state"));
    const std::string reencoded = state_document(*plant.controller);
    const std::string expected_text =
        bio::canonical_json(bio::parse_json(source));
    bt::require(
        std::ranges::equal(
            byte_span(reencoded), byte_span(expected_text)),
        "restore+encode reproduces the recorded wire byte-for-byte");
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
    const std::array<double, 2> powers{
        diagnostics.positive_power_w,
        diagnostics.positive_work_step_j.value_or(0.),
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
  // The removed joint_passive_damping_nms_rad knob is an unknown key.
  bio::Value damping_doc = member(doc, "rider_controller");
  std::get<bio::Value::Object>(damping_doc.data)
      .emplace_back("joint_passive_damping_nms_rad",
                    bio::Value{bio::Number{"0.0"}});
  bt::require(throws_std([&] {
                (void)rider::decode_spindle_config(damping_doc);
              }),
              "the removed joint_passive_damping_nms_rad key rejects");
  bt::require(throws_std([&] {
                (void)rider::decode_spindle_config(
                    bio::Value{bio::Value::Object{}});
              }),
              "missing config members reject");
}

// The reposition law takes its gain and torque cap from SpindleConfig
// (2026-11-16 plan task 2): the cap is the rider's crank torque ceiling
// seated_climb.max_crank_torque_nm, the gain is the profile field
// drive.pedaling.reposition_damping_nms_rad. A configured cap of 30 N m
// bounds the saturated backpedal task at 30, and halving the gain halves
// the unsaturated task.
void test_spindle_reposition_gain_and_cap_come_from_config() {
    rider::RiderCommand command;
    command.mean_crank_torque_nm = 0.;
    command.crank_target_phase_rad = 0.;
    command.crank_target_rate_rad_s = -3.;
    command.crank_reposition = true;
    // A fast forward crank drives the raw damper far past the bound, so
    // the delivered task pins the configured cap, not a code constant.
    {
        Plant capped = make_plant({}, [](rider::SpindleConfig &c) {
            c.reposition_torque_limit_nm = 30.;
        });
        stage(capped, .35, 40.);
        (void)capped.controller->compute(capped.data, command, true,
                                         .00125, false);
        const double task = capped.controller->state()
                                .allocation_diagnostics.crank_task_nm;
        bt::require(task == -30.,
                    std::format("a 30 N m configured cap clamps the "
                                "saturated reposition task at -30, got "
                                "{:.6f}", task));
    }
    // At a stopped crank the raw task is gain * (-3 rad/s), below either
    // cap: gain 6 delivers exactly half the gain-12 task.
    {
        Plant twelve = make_plant();
        Plant six = make_plant({}, [](rider::SpindleConfig &c) {
            c.reposition_damping_nms_rad = 6.;
        });
        stage(twelve, .35, 0.);
        stage(six, .35, 0.);
        (void)twelve.controller->compute(twelve.data, command, true,
                                         .00125, false);
        (void)six.controller->compute(six.data, command, true,
                                      .00125, false);
        const double task12 = twelve.controller->state()
                                  .allocation_diagnostics.crank_task_nm;
        const double task6 = six.controller->state()
                                 .allocation_diagnostics.crank_task_nm;
        bt::require(task12 == -36.,
                    std::format("gain 12 over -3 rad/s lands at -36, got "
                                "{:.6f}", task12));
        bt::require(task6 == -18. && task6 == .5 * task12,
                    std::format("gain 6 halves the unsaturated task: "
                                "{:.6f} vs {:.6f}", task6, task12));
    }
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
        test_free_crank_stays_held_under_balanced_leg_weights();
        test_spindle_coasting_applies_no_crank_torque();
        test_spindle_reposition_backpedals_toward_goal();
        test_spindle_reposition_gain_and_cap_come_from_config();
        test_spindle_reposition_command_wire();
        test_spindle_coasting_leg_rests_with_own_weight();
        test_legacy_coasting_leg_rests_with_own_weight();
        test_spindle_compute_matches_python();
        test_spindle_rear_chain_effort_matches_python();
        test_spindle_disabled_command_matches_python();
        test_spindle_steady_state_bypasses_activation();
        test_spindle_coasting_command_matches_python();
        test_spindle_compute_posture_matches_python();
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
