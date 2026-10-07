// writers/rider_contacts.hpp — the model-owned port of RiderContactApplier
// (src/bike_sim/sim/ride/rider_contacts.py), writer core (T3b-1).
//
// Ported in this wave: the two-pad unilateral supports (finite box pads on
// named support geoms, contactlaw::normal_contact/brush_step, tangent
// transport and face-switch handling, paired-reaction generalized force via
// the relative point Jacobian), the releasable spring grip plus its
// cohesive pair overload, the connect-grip diagnostics path, the
// welded/pinned/spindle support diagnostics reading the (still-null,
// wave-3b) settled latch, capture/release through set_enabled, probe
// publication for advance=False, and the serializable state container.
//
// Deliberately absent (wave 3b): settle_welds latch production,
// prepare_attachment_raw, attachment_samples — the latch field exists in
// the snapshot so a restored or future-produced value round-trips, but no
// code path in this TU ever writes one.
//
// Numerical contract: expression order follows rider_contacts.py line by
// line; `a @ b`/`A @ x`/`np.linalg.norm` route through the Accelerate CBLAS
// entry points numpy dispatches to (rider/contact_math.hpp, cblas_abi.hpp),
// `math.hypot` through rider::hypot3, `x**y` through pyfloat::pow, and the
// TU compiles under the module-wide -ffp-contract=off.
#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../rider/attachment_wrench.hpp"
#include "../rider/contact_config.hpp"
#include "../rider/contact_math.hpp"
#include "../rider/equality_reactions.hpp"

namespace writers {

    // rider_contacts.py:54 — CONTACTS, the fixed iteration/serialization
    // order: 0 'saddle', 1 'front_pedal', 2 'rear_pedal', 3 'grip'.
    inline constexpr std::size_t kSupportCount = 3;
    inline constexpr std::size_t kGripIndex = 3;
    inline constexpr std::size_t kContactCount = 4;
    inline constexpr std::size_t kPadsPerSupport = 2;
    inline constexpr std::size_t kPadStateCount =
        kSupportCount * kPadsPerSupport;
    inline constexpr std::size_t kSideCount = 2;
    inline constexpr std::array<std::string_view, kContactCount>
        kContactNames = {"saddle", "front_pedal", "rear_pedal", "grip"};
    inline constexpr std::array<std::string_view, kSupportCount>
        kSupportNames = {"saddle", "front_pedal", "rear_pedal"};
    inline constexpr std::array<std::string_view, kSideCount>
        kSideNames = {"left", "right"};

    // _SupportState (rider_contacts.py:47-50): shear coordinate plus the
    // material tangent it was last transported on (None ↔ nullopt).
    struct RiderContactSupportState {
        double xi = 0.;
        std::optional<rider::Vec3> tangent;
    };

    // ---- the diagnostics dict, typed -----------------------------------
    // Field presence mirrors the oracle's dict.update() writes: a pad
    // support carries 'patches' only when detailed, a welded/pinned
    // support always has would_*/tangent_n and its detailed
    // 'tangent_force_n' is the np.zeros(3) vector, not the scalar.

    struct RiderContactPatchDiag {
        bool in_platform = false;
        double normal_load_n = 0.;
        double gap_m = 0.;
        rider::Vec3 point_m{};
        rider::Vec3 force_on_rider_n{};
        double radial_energy_j = 0.;
        double shear_energy_j = 0.;
    };

    struct RiderContactSupportDiag {
        bool enabled = false;
        bool in_platform = false;
        double normal_load_n = 0.;
        // welded == true selects the weld/pin dict layout: would_separate,
        // would_slip and tangent_n are emitted, and the detailed
        // 'tangent_force_n' is tangent_force_vec (np.zeros(3)); on a pad
        // support the detailed key is the tangent_force_scalar.
        bool welded = false;
        bool would_separate = false;
        bool would_slip = false;
        double tangent_n = 0.;
        double gap_m = 0.;
        double vertical_force_on_rider_n = 0.;
        bool detailed = false;
        double tangent_force_scalar = 0.;
        rider::Vec3 tangent_force_vec{};
        std::vector<RiderContactPatchDiag> patches;
        rider::Vec3 force_on_rider_n{};
        rider::Vec3 force_on_bike_n{};
        rider::Vec3 moment_nm{};
        double radial_energy_j = 0.;
        double shear_energy_j = 0.;
        double relative_power_w = 0.;
    };

    struct RiderContactGripSideDiag {
        // welded == true: the connect-equality layout — base keys
        // {enabled, reachable, hand_gap_m} with `extended` carrying the
        // detailed block. welded == false: the spring layout, whose
        // per-side dict always carries the full extended set.
        bool welded = false;
        bool enabled = false;
        bool reachable = false;
        double hand_gap_m = 0.;
        bool extended = false;
        bool overloaded = false;
        double trial_pair_force_n = 0.;
        std::optional<double> pair_force_limit_n;
        double release_loss_j = 0.;
        double shoulder_distance_m = 0.;
        double arm_reach_m = 0.;
        rider::Vec3 point_m{};
        rider::Vec3 force_on_rider_n{};
        rider::Vec3 force_on_bike_n{};
        double elastic_energy_j = 0.;
    };

    struct RiderContactGripDiag {
        // Same welded/spring split as the side dicts; the aggregate never
        // carries point_m.
        bool welded = false;
        bool enabled = false;
        bool reachable = false;
        double hand_gap_m = 0.;
        bool extended = false;
        bool overloaded = false;
        double trial_pair_force_n = 0.;
        std::optional<double> pair_force_limit_n;
        double release_loss_j = 0.;
        double shoulder_distance_m = 0.;
        double arm_reach_m = 0.;
        rider::Vec3 force_on_rider_n{};
        rider::Vec3 force_on_bike_n{};
        double elastic_energy_j = 0.;
    };

    // self.diagnostics — empty dict until the first evaluation commits.
    struct RiderContactsDiagnostics {
        bool evaluated = false;
        std::array<RiderContactSupportDiag, kSupportCount> supports;
        std::array<RiderContactGripSideDiag, kSideCount> grip_sides;
        RiderContactGripDiag grip;
    };

    // One _settled_welds[0] entry. Support entries carry the five-key dict
    // settle_welds writes; 'grip_*' entries carry force_on_rider_n only
    // (support_fields == false).
    struct RiderSettledEntry {
        bool support_fields = false;
        rider::Vec3 force_on_rider_n{};
        double normal_n = 0.;
        double tangent_n = 0.;
        bool would_separate = false;
        bool would_slip = false;
    };

    // self._settled_welds — the (welds dict, crank) pair settle_welds
    // latches in wave 3b. entries keeps the dict's insertion order.
    struct RiderSettledWelds {
        std::vector<std::pair<std::string, RiderSettledEntry>> entries;
        double crank_torque_nm = 0.;
    };

    // probe_diagnostics / probe_enabled / probe_delivered_crank_torque_nm —
    // published only by an advance=False evaluation.
    struct RiderContactsProbe {
        RiderContactsDiagnostics diagnostics;
        std::array<bool, kContactCount> enabled{};
        double delivered_crank_torque_nm = 0.;
    };

    // The serializable mutable state: every field reset()/compute_qfrc/
    // set_enabled owns, excluding model/data pointers, Jacobian scratch and
    // the equality workspace. Restoring commits this container wholesale.
    struct RiderContactsState {
        std::array<bool, kContactCount> enabled{true, true, true, true};
        std::array<RiderContactSupportState, kPadStateCount> supports;
        std::array<rider::Vec3, kSideCount> grip_xi_local{};
        std::array<std::optional<rider::Vec3>, kSideCount> grip_anchor_local;
        double elastic_energy_j = 0.;
        double loss_step_j = 0.;
        double radial_dissipation_power_w = 0.;
        double delivered_crank_torque_nm = 0.;
        std::optional<double> last_time_s;
        double pending_release_loss_j = 0.;
        RiderContactsDiagnostics diagnostics;
        std::optional<RiderSettledWelds> settled_welds;
        // attachment_raw/sample outputs — empty until wave 3b produces
        // them; typed so a restored snapshot round-trips.
        std::vector<std::pair<std::string, rider::AttachmentSample>>
            attachment_samples;
        std::vector<std::string> attachment_errors;
    };

    class RiderContactWriter {
    public:
        // Resolves every literal the Python ctor resolves (physical_mapping
        // resolve_id messages preserved), selects the attachment link set
        // from the config, and performs reset(model, None) — anchors stay
        // uninitialized until reset(data), like the oracle.
        RiderContactWriter(const mjModel *model,
                           const rider::RiderContactsConfig &config);

        // reset(model, data): planar support topology validation plus
        // spring/connect anchor capture when data is non-null; the full
        // field ordering of rider_contacts.py:98-123, including the
        // partial-state-on-throw semantics (fields above a failing
        // validation stay reset, later fields keep their old values).
        void reset(mjData *data);

        void restart_clock() { state_.last_time_s.reset(); }

        // initialize_settled_state (rider_contacts.py:178-190): one
        // advancing evaluation at the model timestep on a fresh clock,
        // then transient-loss fields zeroed and the clock restarted.
        void initialize_settled_state(mjData *data);

        // set_enabled (rider_contacts.py:125-169): linked-release refusal,
        // pending release-loss accounting (stored xi energy plus the
        // outgoing-pose normal energy), and the grip capture gate.
        [[nodiscard]] bool set_enabled(std::string_view name, bool enabled);
        void release_all();

        // compute_qfrc (rider_contacts.py:425-682): advance commits
        // material state; advance=False evaluates on a private state copy
        // and publishes only the probe view. Writes exactly nv doubles.
        void compute_qfrc_into(mjData *data, double dt, bool advance,
                               bool detailed, std::span<double> out);

        // stored_energy (rider_contacts.py:207-219): interface elastic
        // ledger — grip springs plus enabled in-platform pad normal/shear
        // energy, in the oracle's accumulation order.
        [[nodiscard]] double stored_energy(const mjData *data) const;

        [[nodiscard]] const RiderContactsState &state() const {
            return state_;
        }

        [[nodiscard]] const std::optional<RiderContactsProbe> &
        probe() const {
            return probe_;
        }

        // Wholesale state restore: the binding stages and validates every
        // field before this noexcept commit — no partial mutation.
        void assign_state(RiderContactsState &&state,
                          std::optional<RiderContactsProbe> &&probe) {
            state_ = std::move(state);
            probe_ = std::move(probe);
        }

        [[nodiscard]] const rider::RiderContactsConfig &config() const {
            return cfg_;
        }

        [[nodiscard]] bool linked_pedals() const { return linked_pedals_; }
        [[nodiscard]] bool spindle_pedals() const { return spindle_pedals_; }
        [[nodiscard]] bool linked_saddle() const { return linked_saddle_; }
        [[nodiscard]] bool welded_saddle() const { return welded_saddle_; }
        [[nodiscard]] bool welded_grip() const { return welded_grip_; }
        [[nodiscard]] int saddle_eq() const { return saddle_eq_; }
        [[nodiscard]] int pedal_eq(std::size_t side) const {
            return pedal_eq_.at(side);
        }
        [[nodiscard]] int grip_eq(std::size_t side) const {
            return grip_eq_.at(side);
        }

    private:
        // (rider body, sole site, bike body, support geom) — the
        // self.supports dict entries, CONTACTS order.
        struct SupportRef {
            int body = -1, site = -1, bike = -1, geom = -1;
        };

        struct PadEval {
            rider::Vec3 point{};
            rider::Vec3 normal{};
            rider::Vec3 tangent{};
            double gap = 0.;
            bool inside = false;
        };

        // The advancing/non-advancing shared kernel: evaluate() runs on
        // either the live state or the probe copy and writes the qfrc the
        // call returns. It performs the dt/clock/anchor gates itself, like
        // the oracle's advancing path.
        void evaluate(mjData *data, double dt, bool detailed,
                      RiderContactsState &w, std::span<double> out);

        // _pads (rider_contacts.py:192-205) — pad i of support s at the
        // CURRENT data pose; center = sole site + foot rotation offset,
        // then the compiled-geometry box contact (already validated by
        // reset(), hence the unchecked entry like the oracle's _box_pad_contact).
        void pad_eval(std::size_t support, std::size_t pad,
                      const mjData *data, PadEval &out) const;

        // relative_point_jacobian (physical_mapping.py:40-59): jac_a_ is
        // filled with J_a - J_b at `point`; jac_b_ is scratch. The pair of
        // owned buffers is shared by every call site like the oracle's
        // self._jac_a/self._jac_b.
        const std::vector<double> &
        relative_jacobian(const mjData *data, int body_a, int body_b,
                          const rider::Vec3 &point);

        // _settled(name) (rider_contacts.py:414-419) — (force, max(normal,0))
        // of the wave-3b latch, zeros while it stays null.
        [[nodiscard]] std::pair<rider::Vec3, double>
        settled_force(const RiderContactsState &w, std::string_view name) const;

        [[nodiscard]] const RiderSettledEntry *
        settled_entry(const RiderContactsState &w,
                      std::string_view name) const;

        // Engine-error-capable kinematics refreshes used by set_enabled's
        // release/capture paths (rider_contacts.py:148,157) — invoked
        // through engine::invoke so a fatal MuJoCo error surfaces as
        // engine::EngineFailure and poisons the owning Stepper.
        void refresh_kinematics(mjData *data) const;
        void refresh_kinematics_com(mjData *data) const;

        const mjModel *model_;
        rider::RiderContactsConfig cfg_;
        // Resolved literal ids (RiderContactApplier.__init__, lines 60-81).
        int frame_ = -1;
        int steer_ = -1;
        int pelvis_ = -1;
        int crank_dof_ = -1;
        std::array<int, kSideCount> forearm_{-1, -1};
        std::array<int, kSideCount> shoulder_{-1, -1};
        std::array<int, kSideCount> grip_site_{-1, -1};
        std::array<SupportRef, kSupportCount> supports_{};
        // Attachment discriminators (lines 84-95).
        bool spindle_pedals_ = false;
        bool linked_pedals_ = false;
        bool welded_saddle_ = false;
        bool pinned_saddle_ = false;
        bool linked_saddle_ = false;
        bool welded_grip_ = false;
        // Conditionally resolved equality ids; -1 when the attachment
        // kind does not consume them (the lazy-resolution contract).
        std::array<int, kSideCount> pedal_eq_{-1, -1};
        int saddle_eq_ = -1;
        std::array<int, kSideCount> grip_eq_{-1, -1};
        // Owned per-step scratch — the oracle's self._jac_a/_jac_b plus the
        // np.zeros(nv) qfrc accumulator (sized at construction).
        std::vector<double> jac_a_;
        std::vector<double> jac_b_;
        std::vector<double> qfrc_;
        rider::EqualityReactions equality_;
        // self._data — the last mjData seen by reset/compute_qfrc;
        // set_enabled's capture/release refresh paths read it.
        mjData *seen_ = nullptr;
        RiderContactsState state_;
        // Python keeps probe_diagnostics/probe_enabled/
        // probe_delivered_crank_torque_nm as attributes that only exist
        // after the first advance=False call — nullopt mirrors "unset".
        std::optional<RiderContactsProbe> probe_;
    };
} // namespace writers
