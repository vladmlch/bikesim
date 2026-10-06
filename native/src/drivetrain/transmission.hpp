#pragma once
#include "chain.hpp"
#include <cstdint>
#include <map>
#include <memory>
#include <utility>
#include <variant>

namespace drivetrain {
    using DiagnosticValue = std::variant<std::monostate, double, int, bool, std::string>;
    using Diagnostics = std::map<std::string, DiagnosticValue>;

    struct DataDeleter {
        void operator()(mjData *data) const {
            if (data)
                mj_deleteData(data);
        }
    };

    using OwnedData = std::unique_ptr<mjData, DataDeleter>;

    struct PreparedTransmission {
        double phi{}, time{};
        std::vector<double> jacobian, qpos;
    };

    struct TransmissionSnapshot {
        double ratio{};
        int rear_teeth{};
        std::optional<double> boundary;
        std::optional<PreparedTransmission> prepared;
        Diagnostics diagnostics;
        bool shift_pending{};
        double shift_parameter_work_j{}, shift_constraint_work_j{}, last_tension_n{};
        std::array<double, 2> range{};
        std::vector<double> coefficients;
    };

    class Transmission {
    public:
        Transmission(mjModel *model, GearingConfig gear, bool geometric = false,
                     const char *tendon = "ideal_mid_drive_freehub",
                     const char *driver = "crank_spin",
                     const char *driven = "rear_wheel_spin", const char *front = "crank");

        // Owns mjData scratch and mutates the model's tendon coefficients:
        // copying or moving would leave the source half-configured, so value
        // semantics stay deleted and ownership remains
        // std::unique_ptr<Transmission> (writers/drivetrain.cpp).
        Transmission(const Transmission &) = delete;
        Transmission &operator=(const Transmission &) = delete;
        Transmission(Transmission &&) = delete;
        Transmission &operator=(Transmission &&) = delete;
        ~Transmission() = default;

        void reset(mjData *data);

        void prepare(mjData *data);

        void set_ratio(mjData *data, double ratio);

        std::span<const double> solved(mjData *data);

        double relative_rate(const mjData *data) const;

        TransmissionSnapshot state() const;

        void validate(const TransmissionSnapshot &state) const;

        void restore(TransmissionSnapshot state);

        const Diagnostics &diagnostics() const { return state_.diagnostics; }
        int driven_dof() const { return driven_dof_; }
        // Private FFI regression diagnostics read storage identity, not
        // snapshots: each generation counter bumps when its buffer's
        // data() pointer changes. The prepared buffers are sized at
        // construction and only ever rewritten in place, so a stable
        // nonzero generation across prepares proves no reallocation —
        // no raw address ever crosses the binding.
        const PreparedTransmission *prepared_storage() const {
            if (!geometric_)
                return nullptr;
            sync_prepared_generations();
            return &prepared_storage_;
        }
        [[nodiscard]] std::pair<std::uint64_t, std::uint64_t>
        prepared_generations() const {
            sync_prepared_generations();
            return {jacobian_generation_, qpos_generation_};
        }

    private:
        mjModel *model_ = nullptr;
        GearingConfig gear_{};
        bool geometric_ = false;
        // tendon_/driver_*/driven_* resolve unconditionally in the ctor and
        // front_/rear_/frame_ only for the geometric model; absent_id is the
        // declared "not resolved" state — never the world body's 0.
        int tendon_ = absent_id, driver_qpos_ = absent_id,
                driver_dof_ = absent_id, driven_qpos_ = absent_id,
                driven_dof_ = absent_id, front_ = absent_id, rear_ = absent_id,
                frame_ = absent_id;
        std::vector<int> coefficients_;
        OwnedData constant_, endpoint_;
        GeometryWorkspace geometry_;
        TransmissionSnapshot state_;
        PreparedTransmission prepared_storage_;
        bool prepared_valid_{};
        std::vector<double> force_, multipliers_, displacement_;
        // Storage-identity telemetry synced on prepared_storage() access.
        mutable const double *jacobian_identity_ = nullptr;
        mutable const double *qpos_identity_ = nullptr;
        mutable std::uint64_t jacobian_generation_ = 0;
        mutable std::uint64_t qpos_generation_ = 0;

        void sync_prepared_generations() const {
            const double *jacobian = prepared_storage_.jacobian.data();
            if (jacobian != jacobian_identity_) {
                jacobian_identity_ = jacobian;
                ++jacobian_generation_;
            }
            const double *qpos = prepared_storage_.qpos.data();
            if (qpos != qpos_identity_) {
                qpos_identity_ = qpos;
                ++qpos_generation_;
            }
        }

        double relative(const mjData *data) const;

        double geometry(mjData *data);

        void linearize(mjData *data, double phi);
    };
} // namespace drivetrain
