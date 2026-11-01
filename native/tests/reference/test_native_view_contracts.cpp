// Port of tests/reference/test_native_view_contracts.py — Stepper array
// views are read-only live windows over mjData's fixed buffers; efc_force's
// arena relocates with the constraint set so retained snapshots are owned
// copies; drive storage diagnostics report generations and capacities,
// never addresses; and the drive-core oracle dispatch refuses kinds outside
// the closed set or any call on a Stepper with an installed drive writer —
// before mutating the shared model.
#include <utility>

#include "drivetrain_reference_support.hpp"
#include "support/model_fixture.hpp"

namespace {
namespace dr = drivetrain_reference;
using drivetrain::Transmission;
using drivetrain::TransmissionSnapshot;

constexpr std::string_view one_slide_xml =
    R"(<mujoco><worldbody><body><joint name="slide" type="slide"/><geom size=".1" mass="1"/></body></worldbody></mujoco>)";
constexpr std::string_view contact_xml = R"(<mujoco><option timestep="0.002" gravity="0 0 -9.81"/>
<worldbody><geom name="floor" type="plane" size="5 5 .1"/>
<body name="a" pos="0 0 .15"><freejoint/><geom type="sphere" size=".1" mass="1" condim="3"/></body>
<body name="b" pos="0.05 0 .35"><freejoint/><geom type="sphere" size=".1" mass="1" condim="3"/></body>
</worldbody></mujoco>)";

[[nodiscard]] Stepper one_slide() {
    bt::ModelFixture const fixture(one_slide_xml);
    return Stepper(bike::initialization::capture_model_image(*fixture.model).mjb);
}

[[nodiscard]] bool all_finite(std::span<const double> values) {
    return std::ranges::all_of(values, [](double value) { return std::isfinite(value); });
}

// test_view_props_retain_temporary_stepper_and_stay_read_only +
// test_view_retains_named_stepper_after_deletion — the binding retains the
// Stepper through reference_internal and marks views non-writeable. Native
// expresses the same contract in the type system: every accessor returns
// std::span<const double> (a read-only window), and the owner relationship
// is scoped — a span outlives nothing the caller does not keep alive. The
// two Python refcount records therefore port as the const-view guarantee
// plus the size/finiteness matrix below; interpreter-side lifetime
// retention has no native counterpart and stays a documented note.
void views_are_const_windows() {
  Stepper const stepper = one_slide(); // nq=nv=1, nu=0 — the one-slide fixture.
  static_assert(std::is_const_v<std::remove_reference_t<
                    decltype(*std::declval<Stepper>().qpos().data())>>,
                "qpos is a read-only view");
  const std::array<std::pair<std::span<const double>, std::size_t>, 7> views{
      {{stepper.qpos(), 1},
       {stepper.qvel(), 1},
       {stepper.qacc(), 1},
       {stepper.qfrc_constraint(), 1},
       {stepper.ctrl(), 0},
       {stepper.qfrc_applied(), 1},
       {stepper.actuator_force(), 0}}};
  for (const auto &[view, expected] : views) {
    static_assert(
        std::is_same_v<typename decltype(view)::element_type, const double>,
        "every view exposes const elements");
    bt::require(view.size() == expected && all_finite(view),
                "each view is a finite read-only window of the engine's width");
  }
    // efc_force is the relocatable arena: the port keeps caller-owned
    // copies rather than retained views (see the arena record below).
    const std::vector<double> efc{stepper.efc_force().begin(), stepper.efc_force().end()};
    bt::require(efc.empty(), "no constraints on the one-slide fixture");
}

// test_views_are_live_windows_matching_engine_state — spans fetched before
// set_state track the same buffers bitwise after forward.
void live_windows() {
    bt::ModelFixture fixture(one_slide_xml);
    auto stepper = Stepper(bike::initialization::capture_model_image(*fixture.model).mjb);
    const auto qpos = stepper.qpos(), qvel = stepper.qvel(), qacc = stepper.qacc();
    const std::array state_qpos{.5}, state_qvel{-.25}, warmstart{0.};
    stepper.set_state(state_qpos, state_qvel, std::span<const double>{},
        std::span<const double>{warmstart}, 0.);
    stepper.forward();
    const auto position = model_access::mutable_buffer(fixture.data->qpos, fixture.model->nq);
    const auto velocity = model_access::mutable_buffer(fixture.data->qvel, fixture.model->nv);
    position[0] = .5; velocity[0] = -.25;
    engine::forward(fixture.model.get(), fixture.data.get());
    const auto bitwise = [](std::span<const double> view, const double* engine, std::size_t size) {
        return view.size() == size && std::ranges::equal(std::as_bytes(view),
            std::as_bytes(model_access::readonly_buffer(engine, static_cast<mjtSize>(size))));
    };
    bt::require(bitwise(qpos, fixture.data->qpos, 1) &&
        bitwise(qvel, fixture.data->qvel, 1) &&
        bitwise(qacc, fixture.data->qacc, 1),
        "previously fetched views track the engine buffers bitwise");
}

// test_efc_force_snapshot_survives_arena_growth — the arena relocates with
// the constraint set; a retained copy stays detached while fresh reads
// follow the current arena.
void efc_snapshot() {
  bt::ModelFixture const fixture(contact_xml);
  auto stepper =
      Stepper(bike::initialization::capture_model_image(*fixture.model).mjb);
  bt::require(stepper.efc_force().empty(), "no contacts before the first step");
  for (int i = 0; i < 300 && stepper.efc_force().empty(); ++i)
    stepper.step();
  bt::require(!stepper.efc_force().empty(), "contacts populate efc_force");
  const std::vector<double> snapshot{stepper.efc_force().begin(),
                                     stepper.efc_force().end()};
  bool changed = false;
  for (int i = 0; i < 200 && !changed; ++i) {
    stepper.step();
    const auto current = stepper.efc_force();
    changed =
        !std::ranges::equal(std::as_bytes(current),
                            std::as_bytes(std::span<const double>{snapshot}));
  }
    bt::require(changed, "efc_force arena grows/relocates across the extra steps");
    bt::require(all_finite(snapshot) && all_finite(stepper.efc_force()),
        "retained snapshot and current arena are both finite");
}

// test_prepared_storage_reports_generations_not_addresses — exactly four
// counters, identity-stable across prepares and reset, capacities cover nv.
void prepared_storage() {
    const auto owner = dr::pair("geometric_ideal_mid_drive");
    const auto baseline = dr::storage(*owner);
    const auto& [jacobian_generation, qpos_generation, jacobian_capacity, qpos_capacity] = baseline;
    bt::require(jacobian_generation >= 1 && qpos_generation >= 1,
        "storage generations count data() identity changes");
    bt::require(std::cmp_greater_equal(jacobian_capacity, owner->model()->nv) &&
                    std::cmp_greater_equal(qpos_capacity, owner->model()->nv),
                "prepared capacities cover the model's velocity width");
    for (int i = 0; i < 6; ++i)
        bt::require(dr::storage(*owner) == baseline, "repeated prepares reuse the same storage");
    owner->drive().reset();
    bt::require(dr::storage(*owner) == baseline, "reset reuses the same prepared storage");
}

// test_prepared_storage_none_without_geometric_transmission.
void prepared_storage_none() {
    const auto owner = dr::pair("ideal_mid_drive");
    const auto* transmission = owner->drive().transmission_storage();
    bt::require(transmission == nullptr || transmission->prepared_storage() == nullptr,
        "the non-geometric transmission reports no prepared storage");
}

// The binding's private per-call oracle dispatch (drive_binding.cpp's
// _drive_core), expressed on the pure-core faces it delegates to: the kind
// set is closed, a Stepper with an installed drive writer refuses before
// any model mutation, and unknown operations reject as argument errors.
TransmissionSnapshot drive_core(Stepper& owner, std::string_view kind, // NOLINT(bugprone-easily-swappable-parameters) positional signature mirrors the Python reference API
    std::string_view operation, double ratio = 34. / 24.) {
    if (kind != "ideal_mid_drive" && kind != "geometric_ideal_mid_drive")
        throw std::invalid_argument(
            "_drive_core kind must be 'ideal_mid_drive' or "
            "'geometric_ideal_mid_drive'");
    if (owner.has_drive())
        throw std::logic_error(
            "_drive_core mutates the live model behind the installed "
            "drive writer; use a bare Stepper (no drive config)");
    return owner.mutate([&]() -> TransmissionSnapshot {
        owner.note_posvel_stale();
        const bool geometric = kind == "geometric_ideal_mid_drive";
        Transmission transient(owner.model(),
            {.front_teeth = 34, .rear_teeth = 24, .chain_pitch_m = .0127}, geometric,
            geometric ? "geometric_mid_drive_freehub" : "ideal_mid_drive_freehub");
        if (operation == "reset") transient.reset(owner.data());
        else if (operation == "prepare") transient.prepare(owner.data());
        else if (operation == "ratio") {
            transient.reset(owner.data());
            transient.set_ratio(owner.data(), ratio);
        } else throw std::invalid_argument("core operation");
        return transient.state();
    });
}

// test_drive_core_rejects_unknown_kind_before_mutation — kinds outside the
// closed set and unknown operations reject as argument errors.
void drive_core_rejects() {
    for (const auto kind : {"elastic_chain", "not_a_model"}) {
        auto stepper = dr::bare(dr::model_xml("ideal_mid_drive"));
        dr::error<std::invalid_argument>([&] { (void)drive_core(*stepper, kind, "reset"); },
            "must be 'ideal_mid_drive' or 'geometric_ideal_mid_drive'");
    }
    auto stepper = dr::bare(dr::model_xml("ideal_mid_drive"));
    dr::error<std::invalid_argument>(
        [&] { (void)drive_core(*stepper, "ideal_mid_drive", "not_an_operation"); },
        "core operation");
}

// test_drive_core_kind_agnostic_on_bare_stepper — both ideal kinds drive
// the transient Transmission: the ratio result reports the requested
// ratio and the reset result carries finite coefficients.
void drive_core_bare() {
    for (const auto kind : {"ideal_mid_drive", "geometric_ideal_mid_drive"}) {
        auto stepper = dr::bare(dr::model_xml(kind));
        const auto result = drive_core(*stepper, kind, "ratio", 34. / 28.);
        bt::require(result.ratio == 34. / 28. && !result.coefficients.empty() &&
            all_finite(result.coefficients),
            "the ratio operation reports the applied gear and coefficients");
        const auto reset = drive_core(*stepper, kind, "reset");
        bt::require(all_finite(reset.coefficients),
            "reset republishes finite coefficients");
    }
}

// test_drive_core_rejected_with_installed_drive_writer — both kinds refuse
// before any mutation; the writer state stays bit-identical.
void drive_core_writer() {
    const auto owner = dr::pair();
    bt::require(owner->has_drive(), "the paired stepper carries its writer");
    const auto before = dr::state(*owner);
    dr::error<std::logic_error>([&] { (void)drive_core(*owner, "ideal_mid_drive", "reset"); },
        "installed drive writer");
    dr::error<std::logic_error>([&] { (void)drive_core(*owner, "geometric_ideal_mid_drive", "ratio", 3.); },
        "installed drive writer");
    bt::require(bike::io::recording_json(dr::state(*owner)) ==
        bike::io::recording_json(before),
        "rejection happens before any writer mutation");
}
} // namespace
int main() {
    // cppcheck-suppress throwInEntryPoint // test failures propagate as exceptions into bt::run's FAIL catch — the harness entry-point contract, not an unhandled throw
    return bt::run([] {
        views_are_const_windows();
        live_windows();
        efc_snapshot();
        prepared_storage();
        prepared_storage_none();
        drive_core_rejects();
        drive_core_bare();
        drive_core_writer();
    });
}
