#pragma once
#include "chain.hpp"
#include <map>
#include <memory>
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
        // Private FFI regression diagnostics read storage identity, not snapshots.
        const PreparedTransmission *prepared_storage() const {
            return geometric_ ? &prepared_storage_ : nullptr;
        }

    private:
        mjModel *model_;
        GearingConfig gear_;
        bool geometric_;
        int tendon_, driver_qpos_, driver_dof_, driven_qpos_, driven_dof_, front_{},
                rear_{}, frame_{};
        std::vector<int> coefficients_;
        OwnedData constant_, endpoint_;
        GeometryWorkspace geometry_;
        TransmissionSnapshot state_;
        PreparedTransmission prepared_storage_;
        bool prepared_valid_{};
        std::vector<double> force_, multipliers_, displacement_;

        double relative(const mjData *data) const;

        double geometry(mjData *data);

        void linearize(mjData *data, double phi);
    };
} // namespace drivetrain
