#include "transmission.hpp"
#include "../engaged.hpp"
#include "../engine_call.hpp"
#include "../model_topology.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace drivetrain {
    Transmission::Transmission(mjModel *model, GearingConfig gear, bool geometric,
                               const char *tendon, const char *driver, const char *driven,
                               const char *front)
        : model_(model), gear_(gear), geometric_(geometric),
          tendon_(resolve(model, mjOBJ_TENDON, tendon)), constant_(engine::make_data(model)),
          endpoint_(geometric ? engine::make_data(model) : nullptr), geometry_(model->nv),
          prepared_storage_{
              .phi = 0., .time = 0.,
              .jacobian = std::vector<double>(static_cast<std::size_t>(geometric ? model->nv : 0)),
              .qpos = std::vector<double>(static_cast<std::size_t>(geometric ? model->nv : 0))
          },
          force_(static_cast<std::size_t>(model->nv)),
          multipliers_(static_cast<std::size_t>(model->njmax > 0 ? model->njmax : 0)),
          displacement_(force_.size()) {
        drivetrain::validate(gear_);
        if (!constant_ || (geometric && !endpoint_))
            throw std::runtime_error("mj_makeData");
        const int dj = resolve(model, mjOBJ_JOINT, driver),
                wj = resolve(model, mjOBJ_JOINT, driven);
        if (geometric) {
            topology::joint(model, dj, driver, mjJNT_HINGE, {0., 1., 0.});
            topology::joint(model, wj, driven, mjJNT_HINGE, {0., 1., 0.});
        } else {
            topology::scalar_joint(model, dj, driver);
            topology::scalar_joint(model, wj, driven);
        }
        driver_qpos_ =
                buffer(model->jnt_qposadr, model->njnt)[static_cast<std::size_t>(dj)];
        driver_dof_ = buffer(model->jnt_dofadr, model->njnt)[static_cast<std::size_t>(dj)];
        driven_qpos_ =
                buffer(model->jnt_qposadr, model->njnt)[static_cast<std::size_t>(wj)];
        driven_dof_ = buffer(model->jnt_dofadr, model->njnt)[static_cast<std::size_t>(wj)];
        state_.ratio = static_cast<double>(gear.front_teeth) / gear.rear_teeth;
        state_.rear_teeth = gear.rear_teeth;
        if (geometric) {
            front_ = resolve(model, mjOBJ_BODY, front);
            rear_ = resolve(model, mjOBJ_BODY, "rear_wheel");
            frame_ = resolve(model, mjOBJ_BODY, "frame");
            coefficients_.assign(force_.size(), -1);
        }
        const int start = buffer(model->tendon_adr,
                                 model->ntendon)[static_cast<std::size_t>(tendon_)],
                end = start + buffer(model->tendon_num,
                                     model->ntendon)[static_cast<std::size_t>(tendon_)];
        for (int i = start; i < end; ++i) {
            const auto at = static_cast<std::size_t>(i);
            if (buffer(model->wrap_type, model->nwrap)[at] != mjWRAP_JOINT) {
                if (geometric)
                    throw std::invalid_argument(
                        "geometric reduction must use one fixed scalar-joint tendon");
                continue;
            }
            const int joint = buffer(model->wrap_objid, model->nwrap)[at];
            if (geometric) {
                const auto dof = static_cast<std::size_t>(buffer(
                    model->jnt_dofadr, model->njnt)[static_cast<std::size_t>(joint)]);
                if (coefficients_.at(dof) >= 0)
                    throw std::invalid_argument("duplicate geometric tendon coordinate");
                coefficients_.at(dof) = i;
            } else if (joint == dj)
                coefficients_.push_back(i);
        }
        if (geometric) {
            if (model->nq != model->nv ||
                std::ranges::find(coefficients_, -1) != coefficients_.end())
                throw std::invalid_argument("geometric tendon must contain every scalar "
                    "model coordinate exactly once");
        } else if (coefficients_.size() != 1)
            throw std::invalid_argument("ideal freehub needs one driver joint coefficient");
    }

    double Transmission::relative(const mjData *d) const {
        const auto q = buffer(d->qpos, model_->nq);
        return state_.ratio * q[static_cast<std::size_t>(driver_qpos_)] -
               q[static_cast<std::size_t>(driven_qpos_)];
    }

    double Transmission::relative_rate(const mjData *d) const {
        const auto v = buffer(d->qvel, model_->nv);
        return state_.ratio * v[static_cast<std::size_t>(driver_dof_)] -
               v[static_cast<std::size_t>(driven_dof_)];
    }

    double Transmission::geometry(mjData *d) {
        mj_kinematics(model_, d);
        mj_comPos(model_, d);
        return geometry_.evaluate(model_, d, gear_, front_, rear_, frame_);
    }

    void Transmission::linearize(mjData *d, double phi) {
        const auto q = buffer(d->qpos, model_->nq);
        const double upper = validation::derived(engaged(state_.boundary) - phi + dot(geometry_.jacobian, q), "transmission.upper_bound");
        bool changed = false;
        auto const prm = buffer(model_->wrap_prm, model_->nwrap);
        for (std::size_t i = 0; i < coefficients_.size(); ++i) {
            auto &c = prm[static_cast<std::size_t>(coefficients_[i])];
            if (c != geometry_.jacobian[i]) {
                c = geometry_.jacobian[i];
                changed = true;
            }
        }
        if (changed)
            engine::set_const(model_, constant_.get());
        buffer(model_->tendon_range,
               2 * model_->ntendon)[2 * static_cast<std::size_t>(tendon_) + 1] = upper;
        prepared_storage_.phi = phi;
        prepared_storage_.time = d->time;
        std::ranges::copy(geometry_.jacobian, prepared_storage_.jacobian.begin());
        std::ranges::copy(q, prepared_storage_.qpos.begin());
        prepared_valid_ = true;
    }

    void Transmission::reset(mjData *d) {
        state_.boundary = geometric_ ? geometry(d) : relative(d);
        if (geometric_) {
            state_.diagnostics.clear();
            state_.shift_pending = false;
            state_.shift_parameter_work_j = state_.shift_constraint_work_j =
                                            state_.last_tension_n = 0.;
            linearize(d, *state_.boundary);
        } else
            buffer(model_->tendon_range,
                   2 * model_->ntendon)[2 * static_cast<std::size_t>(tendon_) + 1] =
                    *state_.boundary;
    }

    void Transmission::prepare(mjData *d) {
        const double phi = geometric_ ? geometry(d) : relative(d);
        state_.boundary = state_.boundary ? std::min(*state_.boundary, phi) : phi;
        if (geometric_)
            linearize(d, phi);
        else
            buffer(model_->tendon_range,
                   2 * model_->ntendon)[2 * static_cast<std::size_t>(tendon_) + 1] =
                    *state_.boundary;
    }

    void Transmission::set_ratio(mjData *d, double ratio) {
        positive(ratio, "gear ratio");
        if (ratio == state_.ratio)
            return;
        if (!geometric_) {
            const double previous = relative(d);
            state_.ratio = ratio;
            buffer(model_->wrap_prm,
                   model_->nwrap)[static_cast<std::size_t>(coefficients_.front())] = ratio;
            engine::set_const(model_, constant_.get());
            const double current = relative(d);
            state_.boundary =
                    state_.boundary ? *state_.boundary + current - previous : current;
            buffer(model_->tendon_range,
                   2 * model_->ntendon)[2 * static_cast<std::size_t>(tendon_) + 1] =
                    *state_.boundary;
            return;
        }
        const double teeth = std::nearbyint(gear_.front_teeth / ratio);
        if (teeth < 3. || teeth > std::numeric_limits<int>::max() ||
            std::abs(ratio - gear_.front_teeth / teeth) >
            1e-12 * std::abs(gear_.front_teeth / teeth))
            throw std::invalid_argument(
                "geometric ratio must identify an integer rear sprocket");
        const double old_phi = geometry(d),
                old_gap = state_.boundary ? *state_.boundary - old_phi : 0.;
        gear_.rear_teeth = static_cast<int>(teeth);
        state_.rear_teeth = gear_.rear_teeth;
        state_.ratio = ratio;
        const double phi = geometry(d);
        state_.boundary = phi + old_gap;
        const double new_gap = *state_.boundary - phi;
        state_.shift_parameter_work_j += state_.last_tension_n * (new_gap - old_gap);
        state_.shift_pending = true;
        linearize(d, phi);
    }

    std::span<const double> Transmission::solved(mjData *d) {
        std::ranges::fill(force_, 0.);
        multipliers_.assign(static_cast<std::size_t>(d->nefc), 0.);
        const auto type = buffer(d->efc_type, d->nefc), ids = buffer(d->efc_id, d->nefc);
        const auto ef = buffer(d->efc_force, d->nefc);
        double tension = 0.;
        bool selected = false;
        for (std::size_t i = 0; i < multipliers_.size(); ++i)
            if (type[i] == mjCNSTR_LIMIT_TENDON && ids[i] == tendon_) {
                multipliers_[i] = ef[i];
                tension += ef[i];
                selected = true;
            }
        if (selected)
            mj_mulJacTVec(model_, d, force_.data(), multipliers_.data());
        if (!geometric_)
            return force_;
        if (!prepared_valid_)
            throw std::runtime_error("prepare the geometric transmission before solving");
        const auto &p = prepared_storage_;
        std::ranges::copy(buffer(d->qpos, model_->nq), endpoint_->qpos);
        const double endpoint = geometry(endpoint_.get());
        const auto q = buffer(d->qpos, model_->nq);
        for (std::size_t i = 0; i < displacement_.size(); ++i)
            displacement_[i] = q[i] - p.qpos[i];
        const double defect = endpoint - p.phi - dot(p.jacobian, displacement_),
                work = dot(force_, displacement_);
        if (state_.shift_pending)
            state_.shift_constraint_work_j += work;
        double error = 0.;
        for (std::size_t i = 0; i < force_.size(); ++i)
            error = std::max(
                error, std::abs(force_[i] - (-std::max(0., tension) * p.jacobian[i])));
        state_.diagnostics = {
            {"transmission_phi_m", p.phi},
            {"transmission_boundary_m", engaged(state_.boundary)},
            {"transmission_gap_m", engaged(state_.boundary) - p.phi},
            {"transmission_tension_n", tension},
            {"transmission_constraint_defect_m", defect},
            {"transmission_interval_work_j", work},
            {"transmission_reaction_error_n", error},
            {"shift_parameter_work_j", state_.shift_parameter_work_j},
            {"shift_interval_constraint_work_j", state_.shift_pending ? work : 0.},
            {"shift_constraint_work_cumulative_j", state_.shift_constraint_work_j},
            {
                "transmission_reference_status",
                std::string("experimental_geometric_reduction")
            }
        };
        state_.last_tension_n = tension;
        state_.shift_pending = false;
        return force_;
    }

    TransmissionSnapshot Transmission::state() const {
        auto s = state_;
        if (prepared_valid_)
            s.prepared = prepared_storage_;
        const auto range = buffer(model_->tendon_range, 2 * model_->ntendon)
                .subspan(2 * static_cast<std::size_t>(tendon_), 2);
        s.range = {range[0], range[1]};
        s.coefficients.clear();
        for (int const index: coefficients_)
            s.coefficients.push_back(
                buffer(model_->wrap_prm, model_->nwrap)[static_cast<std::size_t>(index)]);
        return s;
    }

    void Transmission::validate(const TransmissionSnapshot &s) const {
        positive(s.ratio, "transmission ratio");
        if (s.rear_teeth < 3)
            throw std::invalid_argument("transmission rear teeth");
        finite_optional(s.boundary, "transmission boundary");
        finite(s.shift_parameter_work_j, "shift parameter work");
        finite(s.shift_constraint_work_j, "shift constraint work");
        finite(s.last_tension_n, "last tension");
        if (s.coefficients.size() != coefficients_.size())
            throw std::invalid_argument("transmission coefficient width");
        for (double const c: s.coefficients)
            finite(c, "transmission coefficient");
        for (double const r: s.range)
            finite(r, "transmission range");
        if (s.prepared) {
            const auto &p = *s.prepared;
            if (!geometric_ || !s.boundary || p.qpos.size() != force_.size() ||
                p.jacobian.size() != force_.size())
                throw std::invalid_argument("prepared transmission width");
            finite(p.phi, "prepared phi");
            finite(p.time, "prepared time");
            for (double const x: p.qpos)
                finite(x, "prepared qpos");
            for (double const x: p.jacobian)
                finite(x, "prepared Jacobian");
        }
    }

    void Transmission::restore(TransmissionSnapshot s) {
        validate(s);
        // Widths are already checked. Copy into construction-owned buffers without
        // allocations; snapshot buffers remain owning and detached at the FFI edge.
        prepared_valid_ = s.prepared.has_value();
        if (s.prepared) {
            const auto &prepared = *s.prepared;
            prepared_storage_.phi = prepared.phi;
            prepared_storage_.time = prepared.time;
            std::ranges::copy(prepared.jacobian, prepared_storage_.jacobian.begin());
            std::ranges::copy(prepared.qpos, prepared_storage_.qpos.begin());
        }
        s.prepared.reset();
        state_ = std::move(s);
        const auto &restored = state_;
        gear_.rear_teeth = restored.rear_teeth;
        auto const prm = buffer(model_->wrap_prm, model_->nwrap);
        for (std::size_t i = 0; i < coefficients_.size(); ++i)
            prm[static_cast<std::size_t>(coefficients_[i])] = restored.coefficients[i];
        engine::set_const(model_, constant_.get());
        std::ranges::copy(restored.range,
                          buffer(model_->tendon_range, 2 * model_->ntendon)
                          .subspan(2 * static_cast<std::size_t>(tendon_), 2)
                          .begin());
    }
} // namespace drivetrain
